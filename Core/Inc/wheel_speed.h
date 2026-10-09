/**
 * Front wheel speed: 20-tooth wheels on PB0 (TIM3_CH3) and PB1 (TIM3_CH4), input capture at 1 MHz.
 * TIM3 is 16-bit; overflows are counted in software so every edge gets a 32-bit microsecond timestamp.
 */
#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

typedef struct {
    float    rpm;                   // mean over the teeth of the last update window, 0 = standstill
    uint32_t edge_count;            // accepted teeth since boot
    uint32_t rejected_count;        // edges rejected as noise or missed teeth
    uint32_t last_period_us;        // last tooth period used as reference
    uint32_t last_edge_us;          // timestamp of the last accepted edge
    uint32_t window_period_sum_us;  // accepted periods since the last update
    uint32_t window_edge_count;     // accepted teeth since the last update
    uint8_t  has_last_edge;         // 0 after boot and after a standstill
} WheelSpeed;

// [0] = PB0 (TIM3_CH3), sent as FRONT_LEFT; [1] = PB1 (TIM3_CH4), sent as FRONT_RIGHT
extern WheelSpeed wheel_speed[2];

void    wheel_speed_start(void);                    // after MX_TIM3_Init()
void    wheel_speed_capture(int wheel, uint16_t capture);   // HAL_TIM_IC_CaptureCallback (TIM3)
void    wheel_speed_timer_overflow(void);           // HAL_TIM_PeriodElapsedCallback (TIM3)
void    wheel_speed_update(void);                   // every 10 ms, from the TIM7 interrupt
uint8_t wheel_speed_self_check(void);               // 1 = rpm calculation OK, run once at boot

#endif /* WHEEL_SPEED_H */
