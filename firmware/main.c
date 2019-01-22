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

static SerialConfig sdcfg = {
      .speed = 115200,
      .cr1 = 0,
      .cr2 = USART_CR2_LINEN,
      .cr3 = 0
    };

SerialDriver            *debug_sd   = &SD4;
BaseSequentialStream    *dstr;

int main(void)
{
    /* RT Core initialization */
    chSysInit();
    /* HAL (Hardware Abstraction Layer) initialization */
    halInit();

    // sdStart( debug_sd, &sdcfg );
    // palSetPadMode( GPIOC, 10, PAL_MODE_ALTERNATE(8) );      // TX
    // palSetPadMode( GPIOC, 11, PAL_MODE_ALTERNATE(8) );       // RX
    // dstr = (BaseSequentialStream *)debug_sd;

    ros_driver_init( NORMALPRIO - 1 );
    motors_init();

    chThdCreateStatic(waThread, sizeof(waThread), NORMALPRIO, Thread, NULL /* arg is NULL */);

    while (true)
    {
        palToggleLine( LINE_LED1 );
        chThdSleepSeconds(1);
    }
}
