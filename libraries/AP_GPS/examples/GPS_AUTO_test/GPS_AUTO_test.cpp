/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
     Test for AP_GPS_AUTO
*/

#include <AP_HAL/AP_HAL.h>                                      //This is a common Hardware Abstraction Layer.
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/AP_BoardLED.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <SITL/SITL.h>
#include <AP_Scheduler/AP_Scheduler.h>

void setup();                                                   //This function is defined in most of the libraries. This function is called only once at boot up time. This function is called by main() function in HAL.
void loop();                                                    //This function is defined in most of the libraries. This function is called by main function in HAL. The main work of the sketch is typically in this function only.

const AP_HAL::HAL& hal = AP_HAL::get_HAL();                     //Declare "hal" reference variable. This variable is pointing to Ap_HAL::HAL class's object. Here, AP_HAL is library and HAL is a class in that library. This reference variable can be used to get access to hardware specific functions.                     

static AP_BoardConfig board_config;

#if AP_NOTIFY_GPIO_LED_3_ENABLED
// create board led object
AP_BoardLED board_led;
#endif

// create fake gcs object
GCS_Dummy _gcs;                                                 //gcs stands for Ground Control Station

#if AP_SIM_ENABLED
SITL::SIM sitl;
AP_Baro baro;
AP_Scheduler scheduler;
#endif

// This example uses GPS system. Create it.
static AP_GPS gps;
// Serial manager is needed for UART communications
static AP_SerialManager serial_manager;

//to be called only once on boot for initializing objects
void setup()
{
    hal.console->printf("GPS AUTO library test\n");

    board_config.init();

#if AP_NOTIFY_GPIO_LED_3_ENABLED
    // Initialise the leds
    board_led.init();
#endif

    // Initialize the UART for GPS system
    serial_manager.init();
    gps.init();
}


/*
  print a int32_t lat/long in decimal degrees
 */
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon);
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon)
{
    int32_t dec_portion, frac_portion;
    int32_t abs_lat_or_lon = labs(lat_or_lon);                  //The labs() function in C++ returns the absolute value of a long or long int data.

    // extract decimal portion (special handling of negative numbers to ensure we round towards zero)
    dec_portion = abs_lat_or_lon / 10000000UL;

    // extract fractional portion
    frac_portion = abs_lat_or_lon - dec_portion*10000000UL;

    // print output including the minus sign
    if( lat_or_lon < 0 ) {
        s->printf("-");
    }
    s->printf("%ld.%07ld",(long)dec_portion,(long)frac_portion);
}

// loop
void loop()
{
    static uint32_t last_msg_ms;

    // Update GPS state based on possible bytes received from the module.
    gps.update();

    // If new GPS data is received, output its contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    if (last_msg_ms != gps.last_message_time_ms()) {
        // Reset the time of message
        last_msg_ms = gps.last_message_time_ms();

        // Acquire location
        const Location &loc = gps.location();

        // Print the contents of message
        hal.console->printf("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->printf(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            (double)(loc.alt * 0.01f),
                            (double)gps.ground_speed(),
                            (int)gps.ground_course(),
                            gps.num_sats(),
                            gps.time_week(),
                            (long unsigned int)gps.time_week_ms(),
                            gps.status());
    }

    // Delay for 10 mS will give us 100 Hz invocation rate
    hal.scheduler->delay(10);
}

// Register above functions in HAL board level
AP_HAL_MAIN();
