#ifndef STRIKER_CONFIG_H
#define STRIKER_CONFIG_H

/*
 * motor_control(A, B, C, D), each output independently configured.
 * ARM: 0=unused, 1=left, 2=right. Multiple outputs may belong to one side.
 * COUNTS_PER_REV: Read_Encoder() counts per ACTUATOR OUTPUT shaft revolution.
 * PWM_SIGN: +1/-1 giving the required mechanical direction.
 * ENCODER_SIGN: +1/-1 making that mechanical direction count positively.
 * Confirmed encoder driver: TI12 quadrature with no division in Read_Encoder.
 * Actuator: 7 PPR on motor shaft, 59:1 gearbox => 1652 counts/output revolution.
 */
#ifndef STRIKER_A_ARM
#define STRIKER_A_ARM 1
#endif
#define STRIKER_ENCODER_PPR 7U
#define STRIKER_GEAR_RATIO 59U
#define STRIKER_ENCODER_MULTIPLIER 4U
#define STRIKER_COUNTS_PER_OUTPUT_REV \
    (STRIKER_ENCODER_PPR * STRIKER_GEAR_RATIO * STRIKER_ENCODER_MULTIPLIER)

#ifndef STRIKER_A_COUNTS_PER_REV
#define STRIKER_A_COUNTS_PER_REV STRIKER_COUNTS_PER_OUTPUT_REV
#endif
#ifndef STRIKER_A_PWM_SIGN
#define STRIKER_A_PWM_SIGN -1
#endif
#ifndef STRIKER_A_ENCODER_SIGN
#define STRIKER_A_ENCODER_SIGN -1
#endif

#ifndef STRIKER_B_ARM
#define STRIKER_B_ARM 2
#endif
#ifndef STRIKER_B_COUNTS_PER_REV
#define STRIKER_B_COUNTS_PER_REV STRIKER_COUNTS_PER_OUTPUT_REV
#endif
#ifndef STRIKER_B_PWM_SIGN
#define STRIKER_B_PWM_SIGN 1
#endif
#ifndef STRIKER_B_ENCODER_SIGN
#define STRIKER_B_ENCODER_SIGN 1
#endif

#ifndef STRIKER_C_ARM
#define STRIKER_C_ARM 0
#endif
#ifndef STRIKER_C_COUNTS_PER_REV
#define STRIKER_C_COUNTS_PER_REV 0U
#endif
#define STRIKER_C_PWM_SIGN 1
#define STRIKER_C_ENCODER_SIGN 1

#ifndef STRIKER_D_ARM
#define STRIKER_D_ARM 0
#endif
#ifndef STRIKER_D_COUNTS_PER_REV
#define STRIKER_D_COUNTS_PER_REV 0U
#endif
#define STRIKER_D_PWM_SIGN 1
#define STRIKER_D_ENCODER_SIGN 1

#define STRIKER_TARGET_REVOLUTIONS 10U
#define STRIKER_RUN_PWM 700U
#define STRIKER_MIN_PWM 200U
#define STRIKER_PWM_LIMIT 1000U
#define STRIKER_STALL_MS 2000U
#define STRIKER_MAX_RUN_MS 15000U
#define STRIKER_SETTLE_MS 200U
#define STRIKER_MAX_SETTLE_MS 2000U
#define STRIKER_REVERSE_TOLERANCE_COUNTS 8U

#endif
