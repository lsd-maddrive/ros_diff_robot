#ifndef PROTOS_H_
#define PROTOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>	

#include <ch.h>
#include <hal.h>
#include <chprintf.h>
// #define dprintf(str) {chprintf(dstr, str); chThdSleepMilliseconds(100); }
#define dprintf(str)

/*
 * Configuration
 */

extern BaseSequentialStream    *dstr;

#define PWM_LIMITED_CONTROL

#define ADC_DEBUG_SEND_DATA_NONE        -1
#define ADC_DEBUG_SEND_DATA_FULL        0
#define ADC_DEBUG_SEND_DATA_FILTERED    1

#define ADC_DEBUG_SEND_DATA             ADC_DEBUG_SEND_DATA_FULL
//#define FILTER_FOR_2_SENSORS
#define MAX_SPEED
#define ROS_ENABLED
#define ADC_MATLAB_DEBUG

#ifdef ADC_MATLAB_DEBUG
  #define ADC_MATLAB_DEBUG_LED                LINE_LED2
#endif


//#define  ODOMETRY_DEBUG_OUTPUT

//#define ADC_REPLACE_2ND_CHANNEL_VREFINT
//#define ADC_MATLAB_DEBUG_LINE_SEND

/*
 * Macro functions
 */

#define CROPVAL(x, min, max) ( (x) > (max) ? (max) : \
                               (x) < (min) ? (min) : (x) )

/*
 * Odometry
 */

typedef struct
{
	float x, y, dir;
} odometry_pose_t;

void odometry_init( void );
void odometry_reset( void );
odometry_pose_t *odometry_get_pose( void );


/*
 * ROS
 */

void ros_driver_init( tprio_t prio );
void ros_driver_send_odom_speed( int32_t left, int32_t right );
void ros_driver_send_odometry( int32_t left, int32_t right );
void ros_driver_send_pose( odometry_pose_t *ptr );

/*
 * Motors
 */

void motors_init( void );
void motors_set_left_power( int power );
void motors_set_right_power( int power );

/*
 * Encoders
 */

void encoders_init( void );
void encoders_reset( void );
int32_t encoders_get_left_value( void );
int32_t encoders_get_right_value( void );
int32_t encoders_get_left_speed( void );
int32_t encoders_get_right_speed( void );

#ifdef __cplusplus
}
#endif

#endif /* PROTOS_H_ */
