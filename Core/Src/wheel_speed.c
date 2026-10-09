/**
 * Front wheel speed from tooth periods.
 *
 * - Every edge gets a 32-bit microsecond timestamp (TIM3 capture + software overflow count),
 *   so slow wheels never wrap into fake high speeds.
 * - Noise rejection: a period under MINIMUM_PERIOD_US or under half the previous one is a spike
 *   between two teeth and is ignored; a period over twice the previous one (missed tooth, restart)
 *   only becomes the new reference.
 * - rpm is the mean of all teeth accepted since the last update (every 10 ms).
 * - With no new tooth, rpm can't be higher than one tooth per elapsed time: it decays and drops
 *   to 0 below MINIMUM_RPM (no tooth for 300 ms).
 *
 * Capture, overflow and update all run at the same interrupt priority (TIM3 and TIM7, both 0),
 * so they never preempt each other.
 */
#include "wheel_speed.h"
#include "tim.h"

#define TEETH_PER_REVOLUTION    20.0f
#define MICROSECONDS_PER_MINUTE 60000000.0f
#define MAXIMUM_RPM             3000.0f     // above this an edge can only be noise
#define MINIMUM_RPM             10.0f       // below this the wheel is reported as stopped
#define MINIMUM_PERIOD_US       ((uint32_t)(MICROSECONDS_PER_MINUTE / (TEETH_PER_REVOLUTION * MAXIMUM_RPM)))

WheelSpeed wheel_speed[2];
static uint32_t timer_overflows;            // upper 16 bits of the microsecond timestamp

// Upper half of a timestamp whose lower half was just read from TIM3. An overflow that already
// happened but whose interrupt hasn't run yet still has its flag set: count it if the low half is small.
static uint32_t timestamp(uint16_t low) {
    uint32_t high = timer_overflows;
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) && low < 0x8000) {
        high++;
    }
    return high << 16 | low;
}

static void process_edge(WheelSpeed *wheel, uint32_t time_us) {
    if (!wheel->has_last_edge) {
        wheel->last_edge_us  = time_us;
        wheel->has_last_edge = 1;
        return;
    }
    uint32_t period = time_us - wheel->last_edge_us;
    if (period < MINIMUM_PERIOD_US || (wheel->last_period_us && 2 * period < wheel->last_period_us)) {
        wheel->rejected_count++;            // spike between two teeth: keep waiting for the real one
        return;
    }
    wheel->last_edge_us = time_us;
    if (wheel->last_period_us && period > 2 * wheel->last_period_us) {
        wheel->rejected_count++;            // missed tooth or restart: new reference only
        wheel->last_period_us = period;
        return;
    }
    wheel->last_period_us = period;
    wheel->window_period_sum_us += period;
    wheel->window_edge_count++;
    wheel->edge_count++;
}

static void update_wheel(WheelSpeed *wheel, uint32_t now_us) {
    if (wheel->window_edge_count) {
        wheel->rpm = MICROSECONDS_PER_MINUTE * wheel->window_edge_count
                   / (TEETH_PER_REVOLUTION * (float)wheel->window_period_sum_us);
        wheel->window_edge_count    = 0;
        wheel->window_period_sum_us = 0;
    }
    if (!wheel->has_last_edge) {
        wheel->rpm = 0.0f;
        return;
    }
    uint32_t since_edge = now_us - wheel->last_edge_us;
    float ceiling = since_edge ? MICROSECONDS_PER_MINUTE / (TEETH_PER_REVOLUTION * (float)since_edge) : MAXIMUM_RPM;
    if (ceiling < MINIMUM_RPM) {            // standstill: restart from the next edge
        wheel->rpm            = 0.0f;
        wheel->has_last_edge  = 0;
        wheel->last_period_us = 0;
    } else if (wheel->rpm > ceiling) {
        wheel->rpm = ceiling;
    }
}

void wheel_speed_start(void) {
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);     // overflow interrupt extends the timer to 32 bits
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);     // PB0
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);     // PB1
}

void wheel_speed_capture(int wheel, uint16_t capture) {
    process_edge(&wheel_speed[wheel], timestamp(capture));
}

void wheel_speed_timer_overflow(void) {
    timer_overflows++;
}

void wheel_speed_update(void) {
    uint32_t now_us = timestamp(__HAL_TIM_GET_COUNTER(&htim3));
    update_wheel(&wheel_speed[0], now_us);
    update_wheel(&wheel_speed[1], now_us);
}

// Synthetic teeth through the same code: steady 1500 rpm with a noise spike and a missed tooth,
// then 400 ms without teeth must read 0.
uint8_t wheel_speed_self_check(void) {
    WheelSpeed wheel = {0};
    uint32_t t = 0;

    for (int i = 0; i < 10; i++) {
        process_edge(&wheel, t += 2000);    // 2000 us per tooth = 1500 rpm
    }
    process_edge(&wheel, t + 300);          // spike
    process_edge(&wheel, t += 2000);
    process_edge(&wheel, t += 4000);        // missed tooth
    process_edge(&wheel, t += 2000);
    update_wheel(&wheel, t + 100);
    uint8_t ok = wheel.rpm > 1499.0f && wheel.rpm < 1501.0f && wheel.rejected_count == 2;

    update_wheel(&wheel, t + 400000);
    return ok && wheel.rpm == 0.0f;
}
