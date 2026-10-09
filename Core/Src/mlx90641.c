/**
 * MLX90641 16x12 IR array driver (Melexis datasheet rev 005, sections 10-11). Non-blocking.
 *
 * Reads: the 16-bit register address is written by interrupt (I2C_FIRST_FRAME, no STOP) and
 * HAL_I2C_MasterTxCpltCallback starts the read by DMA with a repeated start (I2C_LAST_FRAME).
 * HAL_I2C_Mem_Read_DMA is not used: in this HAL version it sends the address by polling.
 * Writes are 2 bytes by interrupt (HAL_I2C_Mem_Write_IT).
 *
 * mlx90641_task() is a state machine: each call starts or checks one transfer, or converts
 * PIXELS_PER_STEP pixels, so no call blocks for long (see mlx90641.longest_step_us).
 * Any failure or timeout stops I2C3; after a back-off (100 ms, doubling up to 2 s) the bus is
 * recovered (9 SCL clocks + STOP by hand, I2C3 re-initialised) and the sensor set up again.
 */
#include "mlx90641.h"
#include "i2c.h"
#include <math.h>
#include <string.h>

#define MLX_ADDR            (0x33 << 1)     // default 7-bit slave address 0x33 (HAL wants it shifted)
#define MLX_RATE            2               // 0x800D bits 9:7: 2 = 2 Hz, 3 = 4 Hz, 4 = 8 Hz (also the CAN send rate)
#define RETRY_MINIMUM_MS    100             // back-off before the first retry
#define RETRY_MAXIMUM_MS    2000            // back-off ceiling while the sensor keeps failing
#define PIXELS_PER_STEP     48              // conversion split in 4 calls, each well under 2 ms
#define MLX_SCL_PORT        GPIOA
#define MLX_SCL_PIN         GPIO_PIN_8
#define MLX_SDA_PORT        GPIOC
#define MLX_SDA_PIN         GPIO_PIN_9
#define EE(a)               eeprom[(a) - 0x2400]

typedef enum {
    STATE_RESTART,          // waiting for the back-off, then bus recovery + EEPROM read
    STATE_READ_EEPROM,
    STATE_READ_CONTROL,
    STATE_WRITE_CONTROL,
    STATE_READ_STATUS,      // ready: polls the new-data bit
    STATE_READ_FRAME,
    STATE_CLEAR_STATUS,
    STATE_CONVERT,
} State;

typedef enum { TRANSFER_IDLE, TRANSFER_BUSY, TRANSFER_DONE, TRANSFER_FAILED } Transfer;

MLX90641 mlx90641;

static uint16_t eeprom[832];            // calibration, 0x2400..0x273F
static uint16_t ram[448];               // RAM 0x0400..0x05BF: pixel rows of both subpages + auxiliary data
static uint16_t register_value;         // status or control register
static float    temperature[192];       // frame being converted, published in mlx90641.to when complete

static State             state = STATE_RESTART;
static volatile Transfer transfer = TRANSFER_IDLE;
static uint8_t           address_bytes[2], write_bytes[2];
static uint8_t          *read_destination;
static uint16_t          read_length;
static uint32_t          transfer_start_ms, transfer_timeout_ms;
static uint32_t          restart_since_ms, restart_wait_ms = 80;   // datasheet 10.5: 80 ms after POR
static uint32_t          retry_delay_ms = RETRY_MINIMUM_MS;
static int               next_pixel;

// ---------------- transfers ----------------

static void begin_transfer(uint16_t bytes) {
    transfer            = TRANSFER_BUSY;
    transfer_start_ms   = HAL_GetTick();
    transfer_timeout_ms = 10 + bytes / 5;   // 10 ms + 0.2 ms per byte, ~2x the time at 100 kHz
}

// A held bus fails at once here instead of in HAL's 25 ms busy wait
static void start_read(uint16_t address, uint16_t *destination, uint16_t words) {
    address_bytes[0] = address >> 8;
    address_bytes[1] = address & 0xFF;
    read_destination = (uint8_t *)destination;
    read_length      = words * 2;
    begin_transfer(read_length + 2);
    if (__HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY)
        || HAL_I2C_Master_Seq_Transmit_IT(&hi2c3, MLX_ADDR, address_bytes, 2, I2C_FIRST_FRAME) != HAL_OK) {
        transfer = TRANSFER_FAILED;
    }
}

static void start_write(uint16_t address, uint16_t value) {
    write_bytes[0] = value >> 8;
    write_bytes[1] = value & 0xFF;
    begin_transfer(4);
    if (__HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY)
        || HAL_I2C_Mem_Write_IT(&hi2c3, MLX_ADDR, address, I2C_MEMADD_SIZE_16BIT, write_bytes, 2) != HAL_OK) {
        transfer = TRANSFER_FAILED;
    }
}

// Address sent: read with a repeated start, by DMA
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c3
        && HAL_I2C_Master_Seq_Receive_DMA(hi2c, MLX_ADDR, read_destination, read_length, I2C_LAST_FRAME) != HAL_OK) {
        transfer = TRANSFER_FAILED;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c3) {
        transfer = TRANSFER_DONE;
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c3) {
        transfer = TRANSFER_DONE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c3) {
        mlx90641.i2c_code = hi2c->ErrorCode;
        transfer = TRANSFER_FAILED;
    }
}

static void swap_words(uint16_t *words, uint16_t count) {     // the bus is big-endian
    for (uint16_t i = 0; i < count; i++) {
        words[i] = __REV16(words[i]);
    }
}

// ---------------- failure and bus recovery ----------------

static void wait_microseconds(uint32_t microseconds) {
    uint32_t start = DWT->CYCCNT, cycles = microseconds * (SystemCoreClock / 1000000);
    while (DWT->CYCCNT - start < cycles) {}
}

// Stops I2C3 (and its DMA/interrupts) so nothing late can land, then waits for the back-off
static void restart_after_failure(uint32_t now) {
    mlx90641.bus = HAL_GPIO_ReadPin(MLX_SCL_PORT, MLX_SCL_PIN) << 1 | HAL_GPIO_ReadPin(MLX_SDA_PORT, MLX_SDA_PIN);
    mlx90641.i2c_err++;
    mlx90641.ready = 0;
    HAL_I2C_DeInit(&hi2c3);
    transfer         = TRANSFER_IDLE;
    state            = STATE_RESTART;
    restart_since_ms = now;
    restart_wait_ms  = retry_delay_ms;
    retry_delay_ms   = retry_delay_ms * 2 > RETRY_MAXIMUM_MS ? RETRY_MAXIMUM_MS : retry_delay_ms * 2;
}

// A transfer cut mid-byte (timeout, MCU/debugger reset) leaves the sensor holding SDA low.
// Take the pins from I2C3, clock SCL 9 times so it finishes the byte and sees a NACK,
// send a STOP, then reset and re-init I2C3 (~250 us).
static void bus_recover(void) {
    GPIO_InitTypeDef g = { .Pin = MLX_SCL_PIN, .Mode = GPIO_MODE_OUTPUT_OD, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_LOW };

    HAL_I2C_DeInit(&hi2c3);
    HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_SET);     // released before switching to GPIO
    HAL_GPIO_WritePin(MLX_SDA_PORT, MLX_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(MLX_SCL_PORT, &g);
    g.Pin = MLX_SDA_PIN;
    HAL_GPIO_Init(MLX_SDA_PORT, &g);

    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_RESET); wait_microseconds(10);
        HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_SET);   wait_microseconds(10);
    }
    HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_RESET); wait_microseconds(10);   // STOP: SDA rises while SCL is high
    HAL_GPIO_WritePin(MLX_SDA_PORT, MLX_SDA_PIN, GPIO_PIN_RESET); wait_microseconds(10);
    HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_SET);   wait_microseconds(10);
    HAL_GPIO_WritePin(MLX_SDA_PORT, MLX_SDA_PIN, GPIO_PIN_SET);   wait_microseconds(10);

    __HAL_RCC_I2C3_FORCE_RESET();       // clears a BUSY flag latched while SDA was low
    __HAL_RCC_I2C3_RELEASE_RESET();
    MX_I2C3_Init();                     // MspInit puts the pins back on AF4 and re-links the DMA
    mlx90641.recover_count++;
}

// ---------------- conversion (section 11.2.2) ----------------

// Two's complement of the low `bits` of v (EEPROM words carry 5 Hamming bits on top, ignored)
static int32_t sext(uint32_t v, int bits) {
    v &= (1u << bits) - 1;
    return v >= (1u << (bits - 1)) ? (int32_t)v - (1 << bits) : (int32_t)v;
}

static struct {             // per-frame values shared by all pixels
    int   subpage;
    float dv, ta, kgain, cp, alphacp, tgc, emis, ksta, tar, osavg, ktaavg, kvavg;
    int   osscale, ktas1, ktas2, kvs1, kvs2;
    float ct[8], ksto[8], acorr[8];
} frame;

// ADC resolution is left at the calibrated value, so Resolution_corr = 1.
static void prepare_frame(int subpage) {
    const uint16_t *aux = ram + 0x180;                                     // RAM 0x0580..0x05BF
    frame.subpage = subpage;

    // 11.2.2.2 Vdd
    float kvdd  = sext(EE(0x2427), 11) * 32.0f;
    float vdd25 = sext(EE(0x2426), 11) * 32.0f;
    frame.dv    = ((int16_t)aux[0x2A] - vdd25) / kvdd;                     // Vdd - 3.3 V (RAM 0x05AA)

    // 11.2.2.3 Ta
    float kvptat  = sext(EE(0x242B), 11) / 4096.0f;
    float ktptat  = sext(EE(0x242A), 11) / 8.0f;
    float vptat25 = 32.0f * (EE(0x2428) & 0x7FF) + (EE(0x2429) & 0x7FF);
    float aptat   = (EE(0x242C) & 0x7FF) / 128.0f;
    float vptat   = (int16_t)aux[0x20];                                    // RAM 0x05A0
    float vbe     = (int16_t)aux[0x00];                                    // RAM 0x0580
    float vart    = vptat / (vptat * aptat + vbe) * 262144.0f;
    frame.ta      = (vart / (1 + kvptat * frame.dv) - vptat25) / ktptat + 25;

    // 11.2.2.4 gain, 11.2.2.6 compensation pixel, 11.2.2.7 TGC, emissivity, KsTa
    frame.kgain   = (32.0f * (EE(0x2424) & 0x7FF) + (EE(0x2425) & 0x7FF)) / (int16_t)aux[0x0A];   // RAM 0x058A
    float cpkta   = ldexpf(sext(EE(0x2431), 6), -((EE(0x2431) & 0x7C0) >> 6));
    float cpkv    = ldexpf(sext(EE(0x2432), 6), -((EE(0x2432) & 0x7C0) >> 6));
    float cpos    = sext(32 * (EE(0x242F) & 0x7FF) + (EE(0x2430) & 0x7FF), 16);
    frame.cp      = (int16_t)aux[subpage ? 0x28 : 0x08] * frame.kgain      // RAM 0x05A8 (SP1) / 0x0588 (SP0)
                  - cpos * (1 + cpkta * (frame.ta - 25)) * (1 + cpkv * frame.dv);
    frame.alphacp = ldexpf(EE(0x242D) & 0x7FF, -(EE(0x242E) & 0x7FF));
    frame.tgc     = sext(EE(0x2433), 9) / 64.0f;
    frame.emis    = sext(EE(0x2423), 11) / 512.0f;
    frame.ksta    = sext(EE(0x2422), 11) / 32768.0f;

    // 11.1.9-11.1.11 temperature ranges: corner temps, KsTo, alpha correction
    static const uint16_t ksto_ee[8] = { 0x2435, 0x2436, 0x2437, 0x2438, 0x2439, 0x243B, 0x243D, 0x243F };
    const float corner[8] = { -40, -20, 0, 80, 120, EE(0x243A) & 0x7FF, EE(0x243C) & 0x7FF, EE(0x243E) & 0x7FF };
    memcpy(frame.ct, corner, sizeof(corner));
    for (int i = 0; i < 8; i++) frame.ksto[i] = ldexpf(sext(EE(ksto_ee[i]), 11), -(EE(0x2434) & 0x7FF));
    frame.acorr[2] = 1;
    frame.acorr[1] = 1 / (1 + frame.ksto[1] * (frame.ct[2] - frame.ct[1]));
    frame.acorr[0] = frame.acorr[1] / (1 + frame.ksto[0] * (frame.ct[1] - frame.ct[0]));
    for (int i = 3; i < 8; i++) {
        frame.acorr[i] = frame.acorr[i - 1] * (1 + frame.ksto[i - 1] * (frame.ct[i] - frame.ct[i - 1]));
    }

    // 11.2.2.9 reflected temperature unknown -> Tr = Ta - 5
    float tak4 = powf(frame.ta + 273.15f, 4), trk4 = powf(frame.ta - 5 + 273.15f, 4);
    frame.tar  = trk4 - (trk4 - tak4) / frame.emis;

    // 11.1.3-11.1.6 offset / Kta / Kv common parts
    frame.osavg   = sext(32 * (EE(0x2411) & 0x7FF) + (EE(0x2412) & 0x7FF), 16);
    frame.osscale = (EE(0x2410) & 0x7E0) >> 5;
    frame.ktaavg  = sext(EE(0x2415), 11);
    frame.kvavg   = sext(EE(0x2417), 11);
    frame.ktas1   = (EE(0x2416) & 0x7E0) >> 5;
    frame.ktas2   = EE(0x2416) & 0x1F;
    frame.kvs1    = (EE(0x2418) & 0x7E0) >> 5;
    frame.kvs2    = EE(0x2418) & 0x1F;
}

static void convert_pixels(int first, int count) {
    for (int n = first; n < first + count; n++) {
        float pixel  = (int16_t)ram[(n / 32) * 64 + frame.subpage * 32 + n % 32];   // rows interleave SP0/SP1 every 32 words (Fig 13)
        int   g      = n / 32;                                             // alpha reference group = 2 rows
        int   ascale = (g & 1 ? EE(0x2419 + g / 2) & 0x1F : (EE(0x2419 + g / 2) & 0x7E0) >> 5) + 20;
        float alpha  = ldexpf((EE(0x2500 + n) & 0x7FF) / 2047.0f * (EE(0x241C + g) & 0x7FF), -ascale);
        float kta    = ldexpf(ldexpf(sext(EE(0x25C0 + n) >> 5, 6), frame.ktas2) + frame.ktaavg, -frame.ktas1);
        float kv     = ldexpf(ldexpf(sext(EE(0x25C0 + n), 5), frame.kvs2) + frame.kvavg, -frame.kvs1);
        float os     = frame.osavg + ldexpf(sext(EE((frame.subpage ? 0x2680 : 0x2440) + n), 11), frame.osscale);

        float vir = pixel * frame.kgain - os * (1 + kta * (frame.ta - 25)) * (1 + kv * frame.dv);   // 11.2.2.5
        vir = (vir - frame.tgc * frame.cp) / frame.emis;                                             // 11.2.2.7
        float ac  = (alpha - frame.tgc * frame.alphacp) * (1 + frame.ksta * (frame.ta - 25));        // 11.2.2.8

        // 11.2.2.9 To in the basic range, then again with the range it falls in (11.2.2.9.1.3)
        float sx = frame.ksto[2] * sqrtf(sqrtf(ac * ac * ac * vir + ac * ac * ac * ac * frame.tar));
        float to = sqrtf(sqrtf(vir / (ac * (1 - frame.ksto[2] * 273.15f) + sx) + frame.tar)) - 273.15f;
        int r = 0;
        while (r < 7 && to >= frame.ct[r + 1]) r++;
        temperature[n] = sqrtf(sqrtf(vir / (ac * frame.acorr[r] * (1 + frame.ksto[r] * (to - frame.ct[r]))) + frame.tar)) - 273.15f;
    }
}

// ---------------- state machine ----------------

static void step(void) {
    uint32_t now = HAL_GetTick();

    if (transfer == TRANSFER_BUSY && now - transfer_start_ms > transfer_timeout_ms) {
        mlx90641.i2c_code = HAL_I2C_ERROR_TIMEOUT;
        transfer = TRANSFER_FAILED;
    }
    if (transfer == TRANSFER_BUSY) {
        return;
    }
    if (transfer == TRANSFER_FAILED) {
        restart_after_failure(now);
        return;
    }
    transfer = TRANSFER_IDLE;

    switch (state) {
    case STATE_RESTART:
        if (now - restart_since_ms < restart_wait_ms) {
            return;
        }
        bus_recover();
        start_read(0x2400, eeprom, 832);
        state = STATE_READ_EEPROM;
        break;

    case STATE_READ_EEPROM:
        swap_words(eeprom, 832);
        start_read(0x800D, &register_value, 1);
        state = STATE_READ_CONTROL;
        break;

    case STATE_READ_CONTROL:
        swap_words(&register_value, 1);
        start_write(0x800D, (register_value & ~0x0380) | (MLX_RATE << 7));
        state = STATE_WRITE_CONTROL;
        break;

    case STATE_WRITE_CONTROL:
        mlx90641.ready = 1;
        retry_delay_ms = RETRY_MINIMUM_MS;
        start_read(0x8000, &register_value, 1);
        state = STATE_READ_STATUS;
        break;

    case STATE_READ_STATUS:
        swap_words(&register_value, 1);
        if (!(register_value & 0x0008)) {                   // bit 3: new subpage in RAM
            start_read(0x8000, &register_value, 1);         // looked at again on the next call
            break;
        }
        start_read(0x0400, ram, 448);
        state = STATE_READ_FRAME;
        break;

    case STATE_READ_FRAME:
        swap_words(ram, 448);
        start_write(0x8000, register_value & ~0x0008);      // clear new-data bit
        state = STATE_CLEAR_STATUS;
        break;

    case STATE_CLEAR_STATUS:
        prepare_frame(register_value & 1);                  // bits 2:0: last measured subpage
        next_pixel = 0;
        state = STATE_CONVERT;
        break;

    case STATE_CONVERT:
        convert_pixels(next_pixel, PIXELS_PER_STEP);
        next_pixel += PIXELS_PER_STEP;
        if (next_pixel < 192) {
            break;
        }
        memcpy(mlx90641.to, temperature, sizeof(temperature));
        mlx90641.ta      = frame.ta;
        mlx90641.vdd     = frame.dv + 3.3f;
        mlx90641.subpage = frame.subpage;
        mlx90641.frames++;
        start_read(0x8000, &register_value, 1);
        state = STATE_READ_STATUS;
        break;
    }
}

// Call every 10 ms from the main loop
void mlx90641_task(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;         // cycle counter: step timing + recovery delays
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;

    step();

    uint32_t microseconds = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000);
    if (microseconds > mlx90641.longest_step_us) {
        mlx90641.longest_step_us = microseconds;
    }
    mlx90641.state = state;
}
