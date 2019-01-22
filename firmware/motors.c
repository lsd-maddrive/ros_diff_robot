#include <protos.h>

PWMDriver *pwmDriver      	= &PWMD4;
PWMConfig pwmConf = {
    .frequency 		= 2000000,
    .period    		= 10000,
    .callback  		= NULL,
    .channels  		= {		/* PD12 - 15 */
						  {.mode = PWM_OUTPUT_ACTIVE_LOW, 	.callback = NULL},
						  {.mode = PWM_OUTPUT_ACTIVE_LOW, 	.callback = NULL},
						  {.mode = PWM_OUTPUT_ACTIVE_LOW, 	.callback = NULL},
						  {.mode = PWM_OUTPUT_ACTIVE_LOW, 	.callback = NULL}
					  },
    .cr2        	= 0,
    .dier       	= 0
};

static int power_2_pwm;

void motors_init( void )
{
	palSetPadMode( GPIOD, 12, PAL_MODE_ALTERNATE(2) );	// PD_12 - PWM4/1
	palSetPadMode( GPIOD, 13, PAL_MODE_ALTERNATE(2) );	// PD_13 - PWM4/2
	palSetPadMode( GPIOD, 14, PAL_MODE_ALTERNATE(2) );	// PD_14 - PWM4/3
	palSetPadMode( GPIOD, 15, PAL_MODE_ALTERNATE(2) );	// PD_15 - PWM4/4

	power_2_pwm = pwmConf.period / 100;

	pwmStart( pwmDriver, &pwmConf );
}

#define PWM_MOTOR_LEFT_POS_IDX 	0
#define PWM_MOTOR_LEFT_NEG_IDX 	1

#define PWM_MOTOR_RIGHT_POS_IDX 2
#define PWM_MOTOR_RIGHT_NEG_IDX 3


void motors_set_left_power( int power )
{
	power = CROPVAL( power, -100, 100 );

	int pwm_value = abs(power * power_2_pwm);

	pwmEnableChannel( pwmDriver, PWM_MOTOR_LEFT_POS_IDX, power > 0 ? pwm_value : 0 );
	pwmEnableChannel( pwmDriver, PWM_MOTOR_LEFT_NEG_IDX, power < 0 ? pwm_value : 0 );
}

void motors_set_right_power( int power )
{
	power = CROPVAL( power, -100, 100 );

	int pwm_value = abs(power * power_2_pwm);

	pwmEnableChannel( pwmDriver, PWM_MOTOR_RIGHT_POS_IDX, power > 0 ? pwm_value : 0 );
	pwmEnableChannel( pwmDriver, PWM_MOTOR_RIGHT_NEG_IDX, power < 0 ? pwm_value : 0 );
}
