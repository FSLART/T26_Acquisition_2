/**
 * MLX90641 16x12 IR array driver (Melexis datasheet rev 005, sections 10-11).
 *
 * Flow: on any I2C failure `ready` drops; every MLX_RETRY_MS the bus is recovered
 * (I2C3 off, SCL clocked by hand, STOP, I2C3 back on) and the sensor re-initialised,
 * until it answers. While ready, the status register is polled and every new subpage
 * (= a full 16x12 frame) is read and converted to degC.
 */
#include "mlx90641.h"
#include "i2c.h"
#include <math.h>

#define MLX_ADDR        (0x33 << 1)     // default 7-bit slave address 0x33 (HAL wants it shifted)
#define MLX_RATE        3               // 0x800D bits 9:7: 2 = 2 Hz, 3 = 4 Hz, 4 = 8 Hz
#define MLX_RETRY_MS    100             // recover + re-init period while the sensor doesn't answer
#define MLX_SCL_PORT    GPIOA
#define MLX_SCL_PIN     GPIO_PIN_8
#define MLX_SDA_PORT    GPIOC
#define MLX_SDA_PIN     GPIO_PIN_9
#define EE(a)           ee[(a) - 0x2400]

MLX90641 mlx90641;
static uint16_t ee[832];                // calibration EEPROM 0x2400..0x273F, read at every init

// Words are big-endian on the bus -> swap after reading.
// HAL timeout covers the whole transfer: the 1664-byte EEPROM read takes ~150 ms at 100 kHz.
static HAL_StatusTypeDef mlx_read(uint16_t addr, uint16_t *buf, uint16_t words) {
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c3, MLX_ADDR, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buf, words * 2, 500);
    for (uint16_t i = 0; i < words; i++) buf[i] = __REV16(buf[i]);
    if (st != HAL_OK) {
        mlx90641.i2c_err++;
        mlx90641.i2c_code = hi2c3.ErrorCode;
        mlx90641.ready = 0;
    }
    return st;
}

static HAL_StatusTypeDef mlx_write(uint16_t addr, uint16_t val) {
    uint8_t b[2] = { val >> 8, val & 0xFF };
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c3, MLX_ADDR, addr, I2C_MEMADD_SIZE_16BIT, b, 2, 100);
    if (st != HAL_OK) {
        mlx90641.i2c_err++;
        mlx90641.i2c_code = hi2c3.ErrorCode;
        mlx90641.ready = 0;
    }
    return st;
}

// A transfer cut mid-byte (timeout, MCU/debugger reset) leaves the sensor holding SDA low.
// Take the pins from I2C3, clock SCL 9 times so it finishes the byte and sees a NACK,
// send a STOP, then reset and re-init I2C3.
static void bus_recover(void) {
    GPIO_InitTypeDef g = { .Pin = MLX_SCL_PIN, .Mode = GPIO_MODE_OUTPUT_OD, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_LOW };

    mlx90641.bus = HAL_GPIO_ReadPin(MLX_SCL_PORT, MLX_SCL_PIN) << 1 | HAL_GPIO_ReadPin(MLX_SDA_PORT, MLX_SDA_PIN);

    HAL_I2C_DeInit(&hi2c3);
    HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_SET);     // released before switching to GPIO
    HAL_GPIO_WritePin(MLX_SDA_PORT, MLX_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(MLX_SCL_PORT, &g);
    g.Pin = MLX_SDA_PIN;
    HAL_GPIO_Init(MLX_SDA_PORT, &g);

    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    }
    HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_RESET); HAL_Delay(1);   // STOP: SDA rises while SCL is high
    HAL_GPIO_WritePin(MLX_SDA_PORT, MLX_SDA_PIN, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(MLX_SCL_PORT, MLX_SCL_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(MLX_SDA_PORT, MLX_SDA_PIN, GPIO_PIN_SET);   HAL_Delay(1);

    __HAL_RCC_I2C3_FORCE_RESET();       // clears a BUSY flag latched while SDA was low
    __HAL_RCC_I2C3_RELEASE_RESET();
    MX_I2C3_Init();                     // MspInit puts the pins back on AF4
    mlx90641.recover_count++;
}

// Read calibration EEPROM and set the refresh rate
static HAL_StatusTypeDef mlx_init(void) {
    uint16_t ctrl;
    if (mlx_read(0x2400, ee, 832) != HAL_OK || mlx_read(0x800D, &ctrl, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    return mlx_write(0x800D, (ctrl & ~0x0380) | (MLX_RATE << 7));
}

// Two's complement of the low `bits` of v (EEPROM words carry 5 Hamming bits on top, ignored)
static int32_t sext(uint32_t v, int bits) {
    v &= (1u << bits) - 1;
    return v >= (1u << (bits - 1)) ? (int32_t)v - (1 << bits) : (int32_t)v;
}

// Raw subpage -> mlx90641.to[][] (section 11.2.2). pix = 192 pixels in pixel order, aux = RAM 0x0580..0x05AF.
// ADC resolution is left at the calibrated value, so Resolution_corr = 1.
static void mlx_calc(const uint16_t *pix, const uint16_t *aux, int sp) {
    // 11.2.2.2 Vdd
    float kvdd  = sext(EE(0x2427), 11) * 32.0f;
    float vdd25 = sext(EE(0x2426), 11) * 32.0f;
    float dv    = ((int16_t)aux[0x2A] - vdd25) / kvdd;                     // Vdd - 3.3 V (RAM 0x05AA)

    // 11.2.2.3 Ta
    float kvptat  = sext(EE(0x242B), 11) / 4096.0f;
    float ktptat  = sext(EE(0x242A), 11) / 8.0f;
    float vptat25 = 32.0f * (EE(0x2428) & 0x7FF) + (EE(0x2429) & 0x7FF);
    float aptat   = (EE(0x242C) & 0x7FF) / 128.0f;
    float vptat   = (int16_t)aux[0x20];                                    // RAM 0x05A0
    float vbe     = (int16_t)aux[0x00];                                    // RAM 0x0580
    float vart    = vptat / (vptat * aptat + vbe) * 262144.0f;
    float ta      = (vart / (1 + kvptat * dv) - vptat25) / ktptat + 25;

    // 11.2.2.4 gain, 11.2.2.6 compensation pixel, 11.2.2.7 TGC, emissivity, KsTa
    float kgain   = (32.0f * (EE(0x2424) & 0x7FF) + (EE(0x2425) & 0x7FF)) / (int16_t)aux[0x0A];   // RAM 0x058A
    float cpkta   = ldexpf(sext(EE(0x2431), 6), -((EE(0x2431) & 0x7C0) >> 6));
    float cpkv    = ldexpf(sext(EE(0x2432), 6), -((EE(0x2432) & 0x7C0) >> 6));
    float cpos    = sext(32 * (EE(0x242F) & 0x7FF) + (EE(0x2430) & 0x7FF), 16);
    float cp      = (int16_t)aux[sp ? 0x28 : 0x08] * kgain                 // RAM 0x05A8 (SP1) / 0x0588 (SP0)
                  - cpos * (1 + cpkta * (ta - 25)) * (1 + cpkv * dv);
    float alphacp = ldexpf(EE(0x242D) & 0x7FF, -(EE(0x242E) & 0x7FF));
    float tgc     = sext(EE(0x2433), 9) / 64.0f;
    float emis    = sext(EE(0x2423), 11) / 512.0f;
    float ksta    = sext(EE(0x2422), 11) / 32768.0f;

    // 11.1.9-11.1.11 temperature ranges: corner temps, KsTo, alpha correction
    static const uint16_t ksto_ee[8] = { 0x2435, 0x2436, 0x2437, 0x2438, 0x2439, 0x243B, 0x243D, 0x243F };
    float ct[8] = { -40, -20, 0, 80, 120, EE(0x243A) & 0x7FF, EE(0x243C) & 0x7FF, EE(0x243E) & 0x7FF };
    float ksto[8], acorr[8];
    for (int i = 0; i < 8; i++) ksto[i] = ldexpf(sext(EE(ksto_ee[i]), 11), -(EE(0x2434) & 0x7FF));
    acorr[2] = 1;
    acorr[1] = 1 / (1 + ksto[1] * (ct[2] - ct[1]));
    acorr[0] = acorr[1] / (1 + ksto[0] * (ct[1] - ct[0]));
    for (int i = 3; i < 8; i++) acorr[i] = acorr[i - 1] * (1 + ksto[i - 1] * (ct[i] - ct[i - 1]));

    // 11.2.2.9 reflected temperature unknown -> Tr = Ta - 5
    float tak4 = powf(ta + 273.15f, 4), trk4 = powf(ta - 5 + 273.15f, 4);
    float tar  = trk4 - (trk4 - tak4) / emis;

    // 11.1.3-11.1.6 offset / Kta / Kv common parts
    float osavg   = sext(32 * (EE(0x2411) & 0x7FF) + (EE(0x2412) & 0x7FF), 16);
    int   osscale = (EE(0x2410) & 0x7E0) >> 5;
    float ktaavg  = sext(EE(0x2415), 11), kvavg = sext(EE(0x2417), 11);
    int   ktas1   = (EE(0x2416) & 0x7E0) >> 5, ktas2 = EE(0x2416) & 0x1F;
    int   kvs1    = (EE(0x2418) & 0x7E0) >> 5, kvs2  = EE(0x2418) & 0x1F;

    for (int n = 0; n < 192; n++) {
        int   g      = n / 32;                                             // alpha reference group = 2 rows
        int   ascale = (g & 1 ? EE(0x2419 + g / 2) & 0x1F : (EE(0x2419 + g / 2) & 0x7E0) >> 5) + 20;
        float alpha  = ldexpf((EE(0x2500 + n) & 0x7FF) / 2047.0f * (EE(0x241C + g) & 0x7FF), -ascale);
        float kta    = ldexpf(ldexpf(sext(EE(0x25C0 + n) >> 5, 6), ktas2) + ktaavg, -ktas1);
        float kv     = ldexpf(ldexpf(sext(EE(0x25C0 + n), 5), kvs2) + kvavg, -kvs1);
        float os     = osavg + ldexpf(sext(EE((sp ? 0x2680 : 0x2440) + n), 11), osscale);

        float vir = (int16_t)pix[n] * kgain - os * (1 + kta * (ta - 25)) * (1 + kv * dv);   // 11.2.2.5
        vir = (vir - tgc * cp) / emis;                                                       // 11.2.2.7
        float ac  = (alpha - tgc * alphacp) * (1 + ksta * (ta - 25));                        // 11.2.2.8

        // 11.2.2.9 To in the basic range, then again with the range it falls in (11.2.2.9.1.3)
        float sx = ksto[2] * sqrtf(sqrtf(ac * ac * ac * vir + ac * ac * ac * ac * tar));
        float to = sqrtf(sqrtf(vir / (ac * (1 - ksto[2] * 273.15f) + sx) + tar)) - 273.15f;
        int r = 0;
        while (r < 7 && to >= ct[r + 1]) r++;
        mlx90641.to[n / 16][n % 16] = sqrtf(sqrtf(vir / (ac * acorr[r] * (1 + ksto[r] * (to - ct[r]))) + tar)) - 273.15f;
    }
    mlx90641.ta  = ta;
    mlx90641.vdd = dv + 3.3f;
}

// Call every 10 ms from the main loop
void mlx90641_task(void) {
    static uint16_t pix[192], aux[48];
    static uint32_t last_try_ms;
    uint16_t status;

    if (!mlx90641.ready) {
        uint32_t now = HAL_GetTick();
        if (now < 80 || now - last_try_ms < MLX_RETRY_MS) {    // datasheet 10.5: 80 ms after POR
            return;
        }
        last_try_ms = now;
        bus_recover();
        mlx90641.ready = mlx_init() == HAL_OK;
        return;
    }

    if (mlx_read(0x8000, &status, 1) != HAL_OK || !(status & 0x0008)) {   // bit 3: new data in RAM
        return;
    }
    int sp = status & 1;                                                   // bits 2:0: last measured subpage
    HAL_StatusTypeDef st = HAL_OK;
    for (int k = 0; k < 6; k++) {                                          // rows interleave SP0/SP1 every 32 words (Fig 13)
        st |= mlx_read(0x0400 + k * 0x40 + sp * 0x20, pix + k * 32, 32);
    }
    st |= mlx_read(0x0580, aux, 48);                                       // Vbe, CP, gain, PTAT, Vdd
    st |= mlx_write(0x8000, status & ~0x0008);                             // clear new-data bit
    if (st != HAL_OK) {
        return;
    }

    mlx_calc(pix, aux, sp);
    mlx90641.subpage = sp;
    mlx90641.frames++;
}
