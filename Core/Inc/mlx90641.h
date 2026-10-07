/**
 * MLX90641 16x12 IR array on I2C3 (PA8 SCL / PC9 SDA), Melexis datasheet rev 005.
 * Call mlx90641_task() every 10 ms; results land in `mlx90641`.
 */
#ifndef MLX90641_H
#define MLX90641_H

#include <stdint.h>

typedef struct {
    float    to[12][16];        // object temperature in degC, [row][col] = datasheet Pix(row+1, col+1)
    float    ta;                // sensor package temperature (degC)
    float    vdd;               // sensor supply (V)
    uint32_t frames;            // frames computed (one per subpage)
    uint32_t i2c_err;           // failed I2C transfers
    uint32_t i2c_code;          // hi2c3.ErrorCode of the last failure: 0x04 NACK, 0x20 timeout (bus stuck),
                                // 0x01 bus error, 0x02 arbitration lost
    uint32_t recover_count;     // bus recoveries done (one before every init attempt)
    uint8_t  bus;               // line levels before the last recovery: SCL << 1 | SDA, 3 = both high (idle)
    uint8_t  ready;             // EEPROM read + refresh rate set, frames are being read
    uint8_t  subpage;           // subpage of the last frame
} MLX90641;

extern MLX90641 mlx90641;

void mlx90641_task(void);

#endif /* MLX90641_H */
