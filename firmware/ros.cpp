#include <ros.h>
#include <protos.h>

/*===========================================================================*/
/* SD relative                                                               */
/*===========================================================================*/

SerialConfig sdcfg = {
      .speed = 115200,
      .cr1 = 0,
      .cr2 = USART_CR2_LINEN,
      .cr3 = 0
    };

SerialDriver    *ros_sd     = &SD5;
BaseChannel     *ros_sd_ptr = (BaseChannel *)ros_sd;

/*===========================================================================*/
/* ROS things                                                                */
/*===========================================================================*/

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>

#include <protos.h>

void (*g_cb_func)(uint16_t speed, uint16_t steer) = NULL;

// Example callback
void topic_cb( const std_msgs::UInt16MultiArray &msg )
{
    if ( msg.data_length != 2 )
        return;

    if ( g_cb_func != NULL )
    {
        g_cb_func( msg.data[0], msg.data[1] );
    }

    // palToggleLine( LINE_LED1 );
}

void motor_right_cb( const std_msgs::Int8 &msg )
{   
    motors_set_right_power( msg.data );
}

void motor_left_cb( const std_msgs::Int8 &msg )
{   
    motors_set_left_power( msg.data );
}

ros::NodeHandle                                 ros_node;

std_msgs::UInt16MultiArray                      u16_arr_msg;
std_msgs::Int32                                 i32_enc_left_msg;
std_msgs::Int32                                 i32_enc_right_msg;
std_msgs::UInt8                                 u8_mode_msg;

ros::Publisher                                  topic_encoder_left("encoder_left", &i32_enc_left_msg);
ros::Publisher                                  topic_encoder_right("encoder_right", &i32_enc_right_msg);
ros::Publisher                                  topic_ranges("ranges_raw", &u16_arr_msg);
ros::Publisher                                  topic_mode("mode", &u8_mode_msg);

ros::Subscriber<std_msgs::Int8>                 topic_motor_left("motor_left", &motor_left_cb);
ros::Subscriber<std_msgs::Int8>                 topic_motor_right("motor_right", &motor_right_cb);

ros::Subscriber<std_msgs::UInt16MultiArray>     topic_control("control_raw", &topic_cb);

/*
 * ROS spin thread - used to receive messages
 */

static THD_WORKING_AREA(waSpinner, 128);
static THD_FUNCTION(Spinner, arg)
{
    (void)arg;
    chRegSetThreadName("Spinner");

    while (true)
    {
        ros_node.spinOnce();
        chThdSleepMilliseconds( 20 );
    }
}

//=======================================================

void ros_driver_set_control_cb( void (*cb_func)(uint16_t speed, uint16_t steer) )
{
    g_cb_func = cb_func;
}

void ros_driver_send_rangefinders( uint16_t *data, uint32_t data_size )
{
    u16_arr_msg.data          = data;
    u16_arr_msg.data_length   = data_size;

    topic_ranges.publish(&u16_arr_msg);
}

void ros_driver_send_mode( uint8_t m_mode )
{
    u8_mode_msg.data = m_mode;

    topic_mode.publish(&u8_mode_msg);
}

void ros_driver_send_odometry( int32_t left, int32_t right )
{
    i32_enc_right_msg.data         = right;
    i32_enc_left_msg.data          = left;

    topic_encoder_right.publish(&i32_enc_right_msg);
    topic_encoder_left.publish(&i32_enc_left_msg);
}

void ros_driver_init( tprio_t prio )
{
    /* Serial driver */
    sdStart( ros_sd, &sdcfg );
    palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );      // TX
    palSetPadMode( GPIOD, 2, PAL_MODE_ALTERNATE(8) );       // RX

    /* ROS setup */
    ros_node.initNode();
    ros_node.setSpinTimeout( 20 );

    /* ROS publishers */
//    ros_node.advertise(topic_ranges);
    ros_node.advertise(topic_encoder_right);
    ros_node.advertise(topic_encoder_left);
    // ros_node.advertise(topic_mode);

    /* ROS subscribers */
    ros_node.subscribe(topic_motor_left);
    ros_node.subscribe(topic_motor_right);
    // ros_node.subscribe(topic_control);

    chThdCreateStatic(waSpinner, sizeof(waSpinner), prio, Spinner, NULL);
}
