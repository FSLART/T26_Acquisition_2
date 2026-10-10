/**
 * Front wheel speed from tooth periods.
 *
 * - Every edge gets a 32-bit microsecond timestamp (TIM3 capture + software overflow count),
 *   so slow wheels never wrap into fake high speeds.
 * - Noise rejection: a period under MINIMUM_PERIOD_US or under half the previous one is a spike
 *   between two teeth and is ignored; a period at least twice the previous one is treated as a gap
 *   and the following normal tooth interval establishes a fresh reference.
 * - rpm is the mean of all teeth accepted since the last update (every 10 ms).
 * - With no new tooth, rpm can't be higher than one tooth per elapsed time, so it decays at once
 *   on a sudden stop, a locked wheel or a lost signal (10 ms -> 300 rpm, 100 ms -> 30 rpm).
 *   A steady 5 RPM (600 ms per tooth) is never cut: the bound only drops below 5 RPM after 600 ms.
 * - Zero after 2.5 minimum-speed tooth periods without a new edge (1.5 seconds).
 *
 * Capture, overflow and update all run at the same interrupt priority (TIM3 and TIM7, both 0),
 * so they never preempt each other.
 */
#include "wheel_speed.h"
#include "tim.h"

#define TEETH_PER_REVOLUTION    20.0f
#define MICROSECONDS_PER_MINUTE 60000000.0f
#define MAXIMUM_RPM             3000.0f     // above this an edge can only be noise
#define MINIMUM_RPM             5.0f        // target minimum measurable wheel speed
#define MINIMUM_PERIOD_US       ((uint32_t)(MICROSECONDS_PER_MINUTE / (TEETH_PER_REVOLUTION * MAXIMUM_RPM)))
#define STOP_TIMEOUT_MULTIPLIER 2.5f
#define STOP_TIMEOUT_US         ((uint32_t)(STOP_TIMEOUT_MULTIPLIER * MICROSECONDS_PER_MINUTE / \
                                  (TEETH_PER_REVOLUTION * MINIMUM_RPM)))

WheelSpeed wheel_speed[2];
// UIF is a single pending bit, so the TIM3 update IRQ must run at least once per 65.536 ms.
static volatile uint32_t timer_overflows;   // upper 16 bits; written by TIM3 update IRQ

// Combine a captured CCR value with the software overflow count. When UIF is pending,
// compare the captured count with the live counter: a capture above CNT belongs to before
// the pending wrap; a capture at/below CNT belongs to after it.
static uint32_t timestamp_from_capture(uint16_t capture) {
    uint32_t high = timer_overflows;
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
        uint16_t live_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        if (capture <= live_count) {
            high++;
        }
    }
    return (high << 16) | capture;
}

// Read CNT and UIF as a coherent time snapshot, including a wrap that occurs
// between the counter read and the status-flag read.
static uint32_t timestamp_now(void) {
    uint32_t high = timer_overflows;
    uint16_t low = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
        uint16_t second_read = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        if (second_read < low) {
            low = second_read;
        }
        high++;
    }
    return (high << 16) | low;
}

static void process_edge(WheelSpeed *wheel, uint32_t time_us) {
    if (!wheel->has_last_edge) {
        wheel->last_edge_us  = time_us;
        wheel->has_last_edge = 1;
        return;
    }
    uint32_t period = time_us - wheel->last_edge_us;
    if (period > STOP_TIMEOUT_US) {
        // Treat this as a fresh start after a stop or long signal loss.
        wheel->rpm = 0.0f;
        wheel->last_edge_us = time_us;
        wheel->last_period_us = 0u;
        wheel->window_period_sum_us = 0u;
        wheel->window_edge_count = 0u;
        return;
    }
    if (period < MINIMUM_PERIOD_US || (wheel->last_period_us && 2 * period < wheel->last_period_us)) {
        wheel->rejected_count++;            // spike between two teeth: keep waiting for the real one
        return;
    }
    wheel->last_edge_us = time_us;
    if (wheel->last_period_us && period >= 2 * wheel->last_period_us) {
        wheel->rejected_count++;            // missed tooth or restart: new reference only
        wheel->last_period_us = 0u;         // accept the next normal interval as a fresh reference
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
    if (since_edge > STOP_TIMEOUT_US) {     // allow multiple minimum-speed tooth periods before declaring stop
        wheel->rpm            = 0.0f;
        wheel->has_last_edge  = 0;
        wheel->last_period_us = 0;
        wheel->window_period_sum_us = 0u;
        wheel->window_edge_count = 0u;
        return;
    }
    if (since_edge) {                       // no tooth yet: at most one tooth per elapsed time
        float ceiling = MICROSECONDS_PER_MINUTE / (TEETH_PER_REVOLUTION * (float)since_edge);
        if (wheel->rpm > ceiling) {
            wheel->rpm = ceiling;
        }
    }
}

void wheel_speed_start(void) {
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);     // overflow interrupt extends the timer to 32 bits
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);     // PB0
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);     // PB1
}

void wheel_speed_capture(int wheel, uint16_t capture) {
    if (wheel < 0 || wheel >= 2) {
        return;
    }
    process_edge(&wheel_speed[wheel], timestamp_from_capture(capture));
}

void wheel_speed_timer_overflow(void) {
    timer_overflows++;
}

void wheel_speed_update(void) {
    uint32_t now_us = timestamp_now();
    update_wheel(&wheel_speed[0], now_us);
    update_wheel(&wheel_speed[1], now_us);
}

// Synthetic teeth through the same code: steady 1500 rpm with a noise spike and a missed tooth,
// then no teeth must time out. Also check the requested 5 RPM lower measurement target.
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

    update_wheel(&wheel, t + 50000u);       // wheel locks: 50 ms without a tooth -> at most 60 rpm
    ok = ok && wheel.rpm < 60.5f;

    update_wheel(&wheel, t + STOP_TIMEOUT_US + 100u);
    ok = ok && wheel.rpm == 0.0f;

    WheelSpeed slow_wheel = {0};
    // Start near UINT32_MAX so these 5 RPM intervals cross the 32-bit time wrap.
    uint32_t slow_time = UINT32_MAX - 2u * 600000u;
    for (int i = 0; i < 6; i++) {
        process_edge(&slow_wheel, slow_time += 600000u); // 600 ms/tooth = 5 RPM
    }
    update_wheel(&slow_wheel, slow_time + 100u);
    uint8_t reads_5_rpm = slow_wheel.rpm > 4.9f && slow_wheel.rpm < 5.1f;
    update_wheel(&slow_wheel, slow_time + 600000u);     // a whole tooth period later: 5 RPM still holds
    reads_5_rpm = reads_5_rpm && slow_wheel.rpm > 4.9f;
    update_wheel(&slow_wheel, slow_time + STOP_TIMEOUT_US + 1u);
    return ok && reads_5_rpm && slow_wheel.rpm == 0.0f;
}
