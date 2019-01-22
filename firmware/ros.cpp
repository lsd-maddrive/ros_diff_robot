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
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

#include <protos.h>

void cmd_vel_cb( const geometry_msgs::Twist &msg )
{
    float lin = msg.linear.x;
    float rot = msg.angular.z;

    if ( lin )
    {
        motors_set_right_power( 50 * lin );
        motors_set_left_power( 50 * lin );
    }
    else if ( rot )
    {
        motors_set_right_power( 50 * rot );
        motors_set_left_power( -50 * rot );
    }
    else
    {
        motors_set_right_power( 0 );
        motors_set_left_power( 0 );
    }
}

void motor_right_cb( const std_msgs::Int8 &msg )
{   
    motors_set_right_power( msg.data );
}

void motor_left_cb( const std_msgs::Int8 &msg )
{   
    motors_set_left_power( msg.data );
}

bool task_triggered = false;
void trigger_task_cb( const std_msgs::UInt8 &msg )
{
    task_triggered = true;
}

ros::NodeHandle                                 ros_node;

std_msgs::Int32                                 i32_enc_left_msg;
std_msgs::Int32                                 i32_enc_right_msg;
std_msgs::Float32                               f32_encspeed_left_msg;
std_msgs::Float32                               f32_encspeed_right_msg;

geometry_msgs::Point32                          odometry_pose;

ros::Publisher                                  topic_encoder_left("encoder_left", &i32_enc_left_msg);
ros::Publisher                                  topic_encoder_right("encoder_right", &i32_enc_right_msg);

ros::Publisher                                  topic_encspeed_left("encspeed_left", &f32_encspeed_left_msg);
ros::Publisher                                  topic_encspeed_right("encspeed_right", &f32_encspeed_right_msg);
ros::Publisher                                  topic_pose("odom_pose", &odometry_pose);

ros::Subscriber<std_msgs::Int8>                 topic_motor_left("motor_left", &motor_left_cb);
ros::Subscriber<std_msgs::Int8>                 topic_motor_right("motor_right", &motor_right_cb);
ros::Subscriber<std_msgs::UInt8>                topic_task_trigger("set_task", &trigger_task_cb);
ros::Subscriber<geometry_msgs::Twist>           topic_cmd("cmd_vel", &cmd_vel_cb);

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

void ros_driver_send_pose( odometry_pose_t *ptr )
{
    odometry_pose.x = ptr->x;
    odometry_pose.y = ptr->y;
    odometry_pose.z = ptr->dir;

    topic_pose.publish(&odometry_pose);
}

void ros_driver_send_odometry( int32_t left, int32_t right )
{
    i32_enc_right_msg.data         = right;
    i32_enc_left_msg.data          = left;

    topic_encoder_right.publish(&i32_enc_right_msg);
    topic_encoder_left.publish(&i32_enc_left_msg);
}

void ros_driver_send_odom_speed( int32_t left, int32_t right )
{
    f32_encspeed_right_msg.data = right;
    f32_encspeed_left_msg.data = left;

    topic_encspeed_right.publish(&f32_encspeed_right_msg);
    topic_encspeed_left.publish(&f32_encspeed_left_msg);
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
    ros_node.advertise(topic_encoder_right);
    ros_node.advertise(topic_encoder_left);
    ros_node.advertise(topic_encspeed_right);
    ros_node.advertise(topic_encspeed_left);
    ros_node.advertise(topic_pose);

    /* ROS subscribers */
    ros_node.subscribe(topic_motor_left);
    ros_node.subscribe(topic_motor_right);
    ros_node.subscribe(topic_task_trigger);
    ros_node.subscribe(topic_cmd);

    chThdCreateStatic(waSpinner, sizeof(waSpinner), prio, Spinner, NULL);
}
