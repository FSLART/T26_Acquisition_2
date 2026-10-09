/**
 * MLX90641 16x12 IR array on I2C3 (PA8 SCL / PC9 SDA), Melexis datasheet rev 005.
 * Non-blocking (I2C by interrupt + DMA). Call mlx90641_task() every 10 ms; results land in `mlx90641`.
 */
#ifndef MLX90641_H
#define MLX90641_H

#include <stdint.h>

typedef struct {
    float    to[12][16];        // object temperature in degC, [row][col] = datasheet Pix(row+1, col+1)
    float    ta;                // sensor package temperature (degC)
    float    vdd;               // sensor supply (V)
    uint32_t frames;            // frames computed (one per subpage)
    uint32_t i2c_err;           // failed or timed-out I2C transfers
    uint32_t i2c_code;          // hi2c3.ErrorCode of the last failure: 0x04 NACK, 0x20 timeout,
                                // 0x01 bus error, 0x02 arbitration lost, 0x10 DMA
    uint32_t recover_count;     // bus recoveries done (one before every init attempt)
    uint32_t longest_step_us;   // longest mlx90641_task() call since boot (must stay well under 2000)
    uint8_t  bus;               // line levels at the last failure: SCL << 1 | SDA, 3 = both high
    uint8_t  ready;             // EEPROM read + refresh rate set, frames are being read
    uint8_t  subpage;           // subpage of the last frame
    uint8_t  state;             // driver state (see State in mlx90641.c)
} MLX90641;

extern MLX90641 mlx90641;

void mlx90641_task(void);

#endif /* MLX90641_H */
