#include <protos.h>

static void left_wheel_a_cb(EXTDriver *extp, expchannel_t channel);
static void right_wheel_a_cb(EXTDriver *extp, expchannel_t channel);
static void left_wheel_b_cb(EXTDriver *extp, expchannel_t channel);
static void right_wheel_b_cb(EXTDriver *extp, expchannel_t channel);

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
        [10] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, left_wheel_a_cb},
        [11] = {EXT_CH_MODE_DISABLED, NULL},
        [12] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, left_wheel_b_cb},
        [13] = {EXT_CH_MODE_DISABLED, NULL},
        [14] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, right_wheel_a_cb},
        [15] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, right_wheel_b_cb},
    }
};

int32_t left_enc_ticks = 0;
int32_t right_enc_ticks = 0;

float left_enc_speed = 0;
float right_enc_speed = 0;

int32_t encoders_get_left_value( void )
{
    return left_enc_ticks;
}

int32_t encoders_get_right_value( void )
{
    return right_enc_ticks;
}

int32_t encoders_get_left_speed( void )
{
    return left_enc_speed;
}

int32_t encoders_get_right_speed( void )
{
    return right_enc_speed;
}

int32_t right_enc_cache = 0;
int32_t left_enc_cache = 0;

GPTDriver *speedTmr = &GPTD3;
int32_t tmr_interval = 10000;

static void speed_tmr_cb ( GPTDriver *speedTmr );

static const GPTConfig speedTmrCfg = {
                                             .frequency      =  100000, // 100 KHz
                                             .callback       =  speed_tmr_cb,
                                             .cr2            =  0,
                                             .dier           =  0U

};

static float msr_2_sec;
static void speed_tmr_cb ( GPTDriver *speedTmr )
{
    speedTmr = speedTmr;

    float right_delta = right_enc_ticks - right_enc_cache;
    right_enc_cache = right_enc_ticks;
    right_enc_speed = right_delta * msr_2_sec;

    float left_delta = left_enc_ticks - left_enc_cache;
    left_enc_cache = left_enc_ticks;
    left_enc_speed = left_delta * msr_2_sec;
}

void encoders_reset( void )
{
    right_enc_ticks = 0;
    left_enc_ticks = 0;

    right_enc_cache = 0;
    left_enc_cache = 0;
}

void encoders_init( void )
{
    extStart( &EXTD1, &extcfg );

    msr_2_sec = speedTmrCfg.frequency / tmr_interval;
    gptStart( speedTmr, &speedTmrCfg );
    gptStartContinuous( speedTmr, tmr_interval );
}


#define LEFT_ENC_A_CH   10
#define LEFT_ENC_B_CH   12
#define RIGHT_ENC_A_CH  14
#define RIGHT_ENC_B_CH  15  

static void left_wheel_a_cb(EXTDriver *extp, expchannel_t channel) 
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

static void right_wheel_a_cb(EXTDriver *extp, expchannel_t channel) 
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) )
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) ) 
            right_enc_ticks++;
        else
            right_enc_ticks--;
    }
    else
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) ) 
            right_enc_ticks--;
        else
            right_enc_ticks++;
    }
}

static void left_wheel_b_cb(EXTDriver *extp, expchannel_t channel) 
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, LEFT_ENC_B_CH ) )
    {
        if ( palReadPad( GPIOE, LEFT_ENC_A_CH ) ) 
            left_enc_ticks--;
        else
            left_enc_ticks++;
    }
    else
    {
        if ( palReadPad( GPIOE, LEFT_ENC_A_CH ) ) 
            left_enc_ticks++;
        else
            left_enc_ticks--;
    }
}

static void right_wheel_b_cb(EXTDriver *extp, expchannel_t channel) 
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) )
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) ) 
            right_enc_ticks--;
        else
            right_enc_ticks++;
    }
    else
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) ) 
            right_enc_ticks++;
        else
            right_enc_ticks--;
    }
}
