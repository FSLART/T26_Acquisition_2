/**
 * MLX90641 matrix -> CAN (DBC data_t26: AQT2_Temperatures_1..24, 0x781..0x798, 8 bytes each).
 *
 * Block k carries 8 pixels of the row-major 16x12 matrix: row k/2, columns (k%2)*8 .. +7.
 * Signal = 1 byte, T_degC = raw + 30 (raw 0 = 30 degC). The generated *_encode() is not used:
 * it wraps above 285 degC instead of saturating.
 *
 * Every new acquisition is frozen and its 24 blocks queued in ascending ID order,
 * always leaving one TX mailbox free for the other periodic frames (AQT2 0x720).
 */
#include "thermal_can.h"
#include "can.h"
#include "mlx90641.h"
#include "data_t26.h"
#include <math.h>

_Static_assert(DATA_T26_AQT2_TEMPERATURES_24_FRAME_ID == DATA_T26_AQT2_TEMPERATURES_1_FRAME_ID + 23,
               "AQT2_Temperatures_1..24 IDs must be contiguous");

void thermal_can_task(void) {
    static uint8_t  raw[24][8];         // frozen acquisition, raw[k] = payload of AQT2_Temperatures_(k+1)
    static uint8_t  block = 24;         // next block to queue, 24 = idle
    static uint32_t frozen_frame;

    if (block == 24) {
        if (mlx90641.frames == frozen_frame) {
            return;                     // no new acquisition
        }
        frozen_frame = mlx90641.frames;
        for (int n = 0; n < 192; n++) { // clamp to 30..285 degC, NaN (bad pixel) -> 0
            raw[n / 8][n % 8] = (uint8_t)(fminf(fmaxf(mlx90641.to[n / 16][n % 16] - 30.0f, 0.0f), 255.0f) + 0.5f);
        }
        block = 0;
    }

    while (block < 24 && HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) {
        CAN_TxHeaderTypeDef h = { .StdId = DATA_T26_AQT2_TEMPERATURES_1_FRAME_ID + block, .IDE = CAN_ID_STD,
                                  .RTR = CAN_RTR_DATA, .DLC = DATA_T26_AQT2_TEMPERATURES_1_LENGTH };
        uint32_t mailbox;
        if (HAL_CAN_AddTxMessage(&hcan1, &h, raw[block], &mailbox) != HAL_OK) {
            break;                      // CAN not running: resume from this block later
        }
        block++;
    }
}
