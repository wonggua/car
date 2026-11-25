#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* 根据实验 17 的转向条件默认黑线为低电平；上车先观察原始读数。 */
#define TRACK_BLACK_LEVEL       1
#define AVOID_ACTIVE_LEVEL      0
#define LIGHT_ACTIVE_LEVEL      0
#define CAR_MIN_SPEED           3000
#define CAR_DEFAULT_SPEED       5000
#define CAR_MAX_SPEED           6000
#define TRACK_MIN_PWM           2500
#define TRACK_DEFAULT_PWM       3000
#define TRACK_MAX_PWM           3500
#define TRACK_TURN_PWM          5000
#define TRACK_INNER_CONFIRM_MS   20U
/* Elapsed times from the last line detection. */
#define TRACK_RECOVERY_MS       250U
#define TRACK_SEARCH_REVERSE_MS 600U
#define TRACK_SEARCH_END_MS     1650U
/* Alarm only; these thresholds never inhibit motor commands. */
#define CAR_LOW_BATTERY_MV       6800U
#define CAR_RECOVER_BATTERY_MV   7000U
#define CAR_SCAN_LEFT_ANGLE      150U
#define CAR_SCAN_RIGHT_ANGLE     30U
#define CAR_SCAN_TIMEOUT_MS      900U
#define CAR_OBSTACLE_MM          250U
#define CAR_SONAR_STALE_MS       250U
#define PS2_DRIVE_POLL_MS        10U
#define PS2_IDLE_POLL_MS         20U
#define PS2_RETRY_MS              5U
#define PS2_STALE_MS             60U
#define CAR_BLUETOOTH_BAUD       9600
#define CAR_CONSOLE_BAUD         9600
#endif
