#include <protos.h>

int32_t enc_right_cache = 0;
int32_t enc_left_cache = 0;

odometry_pose_t pose;

const float ticks_per_rotation = 300;
const float meters_per_rotation = 0.155;
const float meters_per_tick = 0.0005167;
const float wheeltrack = 0.23;

void odometry_init( void )
{
	odometry_reset();
}

void odometry_reset( void )
{
	pose.x = 0;
	pose.y = 0;
	pose.dir = 0;

	enc_right_cache = 0;
	enc_left_cache = 0;	

	encoders_reset();
}

odometry_pose_t *odometry_get_pose( void )
{
	int32_t enc_left_ticks = encoders_get_left_value();
	int32_t enc_right_ticks = encoders_get_right_value();

	float passed_path_right = (enc_left_ticks - enc_left_cache) * meters_per_tick;
	float passed_path_left = (enc_right_ticks - enc_right_cache) * meters_per_tick;

	enc_right_cache = enc_right_ticks;
	enc_left_cache = enc_left_ticks;	

	float full_path = (passed_path_right + passed_path_left) / 2;
	pose.dir = pose.dir + (passed_path_left - passed_path_right) / wheeltrack;
	pose.x = pose.x + full_path * cos( pose.dir );
	pose.y = pose.y + full_path * sin( pose.dir );

	return &pose;
}
