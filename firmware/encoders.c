#include <protos.h>

static void left_wheel_cb(EXTDriver *extp, expchannel_t channel);
static void right_wheel_cb(EXTDriver *extp, expchannel_t channel);

static const EXTConfig extcfg = {
	.channels = 
	{
		[0]  = {EXT_CH_MODE_DISABLED, NULL},
		[1]  = {EXT_CH_MODE_DISABLED, NULL},
		[2]  = {EXT_CH_MODE_DISABLED, NULL},
		[3]  = {EXT_CH_MODE_DISABLED, NULL},
		[4]  = {EXT_CH_MODE_DISABLED, NULL},
		[5]  = {EXT_CH_MODE_DISABLED, NULL},
		[6]  = {EXT_CH_MODE_DISABLED, NULL},
		[7]  = {EXT_CH_MODE_DISABLED, NULL},
		[8]  = {EXT_CH_MODE_DISABLED, NULL},
		[9]  = {EXT_CH_MODE_DISABLED, NULL},
		[10] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, left_wheel_cb},
		[11] = {EXT_CH_MODE_DISABLED, NULL},
		[12] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, NULL},
		[13] = {EXT_CH_MODE_DISABLED, NULL},
		[14] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, right_wheel_cb},
		[15] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, NULL},
	}
};

void encoders_init( void )
{
	extStart( &EXTD1, &extcfg );
}

int32_t left_enc_ticks = 0;
int32_t right_enc_ticks = 0;

#define LEFT_ENC_A_CH	10
#define LEFT_ENC_B_CH	12
#define RIGHT_ENC_A_CH	14
#define RIGHT_ENC_B_CH	15	


static void left_wheel_cb(EXTDriver *extp, expchannel_t channel) 
{
  	extp = extp; channel = channel;

  	if ( palReadPad( GPIOE, LEFT_ENC_A_CH ) )
  	{
  		if ( palReadPad( GPIOE, LEFT_ENC_B_CH ) ) 
  			left_enc_ticks++;
  		else
  			left_enc_ticks--;
  	}
  	else
  	{
  		if ( palReadPad( GPIOE, LEFT_ENC_B_CH ) ) 
  			left_enc_ticks--;
  		else
  			left_enc_ticks++;
  	}
}

static void right_wheel_cb(EXTDriver *extp, expchannel_t channel) 
{
  	extp = extp; channel = channel;

  	if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) )
  	{
  		if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) ) 
  			left_enc_ticks++;
  		else
  			left_enc_ticks--;
  	}
  	else
  	{
  		if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) ) 
  			left_enc_ticks--;
  		else
  			left_enc_ticks++;
  	}
}
