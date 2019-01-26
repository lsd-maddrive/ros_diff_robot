#include <ch.h>
#include <hal.h>

#include <chprintf.h>
#include <protos.h>

static THD_WORKING_AREA(waThread, 128);
static THD_FUNCTION(Thread, arg) 
{
    arg = arg;

    while (true)
    {
        palToggleLine( LINE_LED2 );
        chThdSleepSeconds(1);
    }
}

int main(void)
{
    /* RT Core initialization */
    chSysInit();
    /* HAL (Hardware Abstraction Layer) initialization */
    halInit();

    ros_driver_init( NORMALPRIO - 1 );
    motors_init();
    encoders_init();
    odometry_init();

    chThdCreateStatic(waThread, sizeof(waThread), NORMALPRIO, Thread, NULL /* arg is NULL */);

    systime_t time = chVTGetSystemTimeX();

    while (true)
    {
        time += MS2ST( 20 );

        int32_t enc_left = encoders_get_left_value();
        int32_t enc_right = encoders_get_right_value();

        // ros_driver_send_odometry( enc_left, enc_right );
        
        enc_left = encoders_get_left_speed();
        enc_right = encoders_get_right_speed();        

        // ros_driver_send_odom_speed( enc_left, enc_right );

        odometry_pose_t *pose = odometry_get_pose();
        ros_driver_send_pose( pose );

        chThdSleepUntil(time);

        extern bool task_triggered;
        if ( task_triggered )
        {
            odometry_reset();
            while ( encoders_get_right_value() < 300 )
            {
                motors_set_right_power( 10 );
                chThdSleepMilliseconds( 10 );
            }

            motors_set_right_power( 0 );
            task_triggered = false;
            time = chVTGetSystemTimeX();
        }
    }
}
