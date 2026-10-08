/**
 * MLX90641 16x12 matrix -> CAN (DBC data_t26: AQT2_Temperatures_1..24).
 * Call thermal_can_task() from the main loop as often as possible.
 */
#ifndef THERMAL_CAN_H
#define THERMAL_CAN_H

void thermal_can_task(void);

#endif /* THERMAL_CAN_H */
