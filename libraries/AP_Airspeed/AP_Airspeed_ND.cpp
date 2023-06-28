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
  backend driver for airspeed from a I2C NDD0 sensor
 */
#include "AP_Airspeed_ND.h"

#if AP_AIRSPEED_ND_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <utility>
#include <vector>

extern const AP_HAL::HAL &hal;

#define ND_I2C_ADDR1 0x28
#define ND_I2C_ADDR2 0x30

uint8_t DEFAULT_MODE[2] = {0x44, 0x00}; // notch filter disabled, bw limit set to 20Hz-> 63.4Hz odr with auto select, wdg disabled, pressure range set to 0b100

uint8_t MN_ND210[8] = {0x4E, 0x44, 0x32, 0x31, 0x30, 0x00, 0x00, 0x00};
uint8_t MN_ND005D[8] = {0x4E, 0x44, 0x30, 0x30, 0x35, 0x44, 0x00, 0x00};

float nd210_range[7] = {10.0, 5.0, 4.0, 2.0, 1.0, 0.5, 0.25}; // all in inH2O
float nd130_range[6] = {30.0, 20.0, 10.0, 5.0, 4.0, 2.0};
float nd160_range[8] = {60.0, 50.0, 40.0, 30.0, 20.0, 10.0, 5.0, 2.5};
float nd005d_range[6] = {138.4, 110.72, 55.36, 27.68, 22.14, 13.84}; // converted psi to inH2O

AP_Airspeed_ND::AP_Airspeed_ND(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

bool AP_Airspeed_ND::matchModel(uint8_t* reading) {
  
  for (int i = 0; i < 8; i++) {
    if (reading[i] != MN_ND210[i]) {
      goto probeND005;
    }
    _dev_model = DevModel::ND210;
    ::printf("ND210 dev type detected.\n");
    return true;
  }
  probeND005:
  for (int i = 0; i < 8; i++) {
    if (reading[i] != MN_ND005D[i]) {
      return false;
      ::printf("Dev Model not supported.\n");
    }
    _dev_model = DevModel::ND005D;
    ::printf("ND005D dev type detected.\n");
  }
  return true;
}

// probe for a sensor
bool AP_Airspeed_ND::probe(uint8_t bus, uint8_t address)
{
    _dev = hal.i2c_mgr->get_device(bus, address);
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(5);
    uint8_t reading[12]= {'\0'};
    uint8_t model[8] = {'\0'};
    if(!_dev->read(reading, 12)){
        return false;
    }else{
        for (int i=0; i<12; i++){
            if(i>3){
                model[i-4]= reading[i];
            }
        }
        // ::printf("Model is ");
        // for (int j=0; j<8; j++){
        //     ::printf(" 0x%x ", model[j]);
        // }
        // ::printf("\n");
    }

    return matchModel(model);
}

// probe and initialise the sensor
bool AP_Airspeed_ND::init()
{
    static const uint8_t addresses[] = { ND_I2C_ADDR1, ND_I2C_ADDR2 };
    if (bus_is_confgured()) {
        for (uint8_t addr : addresses) {
            if (probe(get_bus(), addr)) {
                goto found_sensor;
            }
        }
    } else {
        FOREACH_I2C_EXTERNAL(bus) {
            for (uint8_t addr : addresses) {
                if (probe(bus, addr)) {
                    goto found_sensor;
                }
            }
        }
        FOREACH_I2C_INTERNAL(bus) {
            for (uint8_t addr : addresses) {
                if (probe(bus, addr)) {
                    goto found_sensor;
                }
            }
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SST_ND[%u]: no sensor found", get_instance());
    // ::printf("SST_ND[%u]: no sensor found\n", get_instance());
    return false;

found_sensor:
    _dev->set_device_type(uint8_t(DevType::NDxxx));
    set_bus_id(_dev->get_bus_id());
    ::printf("SST_ND[%u]: Found bus %u addr 0x%02x\n", get_instance(), _dev->bus_num(), _dev->get_bus_address());

    // send default configuration
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer(DEFAULT_MODE, 2, nullptr,0);


    switch(_dev_model){
        case DevModel::ND210:
            _available_ranges = 7;
            _range_setting = 3;
            _current_range_val = nd210_range[_range_setting];
            break;
        case DevModel::ND005D:
            _available_ranges = 6;
            _range_setting = 3;
            _current_range_val = nd005d_range[_range_setting];
            break;
        default:
            ::printf("No specific device detected/not supported");
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(200000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_ND::_collect, void));
    return true;
}

/*
  this equation is an inversion of the equation in the
  pressure transfer function figure on page 4 of the datasheet
  
  We negate the result so that positive differential pressures
  are generated when the bottom port is used as the static
  port on the pitot and top port is used as the dynamic port
*/
float AP_Airspeed_ND::_get_pressure(int16_t dp_raw) const
{
    const float inH20_to_Pa = 249.08f;
    const float margin = 29491.2f;

    float diff_press_inH2O  = (dp_raw*_current_range_val)/margin;
    float press  = diff_press_inH2O * inH20_to_Pa;
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_ND::_get_temperature(int8_t dT_int, int8_t dT_frac) const
{
    float temp  = dT_int*1.0 + dT_frac*0.01;
    return temp;
}

// read the values from the sensor
void AP_Airspeed_ND::_collect()
{
    uint8_t data[4]; //2 bytes for pressure and 2 for temperature

    if (!_dev->read(data, sizeof(data))) {
        return;
    }

    int16_t dp_raw;
    dp_raw = (data[0] << 8) + data[1];
    // dp_raw = 0x3FFF & dp_raw;
    // dT_raw = (data[2] << 8) + data[3];
    // dT_raw = (0xFFE0 & dT_raw) >> 5;

    // reject any values that are the absolute minimum or maximums these
    // can happen due to gnd lifts or communication errors on the bus
    // if (dp_raw  == 0xFFFF || dp_raw  == 0 || dT_raw  == 0x7FF || dT_raw == 0) {
    //     return;
    // }

    float press  = _get_pressure(dp_raw);
    float temp  = _get_temperature(data[2], data[3]);
    
    // if (!disable_voltage_correction()) {
    //     _voltage_correction(press, temp);
    // }

    WITH_SEMAPHORE(sem);

    _press_sum += press ;
    _temp_sum += temp;
    _press_count += 1;
    _temp_count += 1;

    _last_sample_time_ms = AP_HAL::millis();
}

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void AP_Airspeed_ND::_voltage_correction(float &diff_press_pa, float &temperature)
{
	const float slope = 65.0f;
	const float temp_slope = 0.887f;

	/*
	  apply a piecewise linear correction within range given by above graph
	 */
	float voltage_diff = hal.analogin->board_voltage() - 5.0f;

    voltage_diff = constrain_float(voltage_diff, -0.7f, 0.5f);

	diff_press_pa -= voltage_diff * slope;
	temperature -= voltage_diff * temp_slope;
}

// return the current differential_pressure in inH2O
bool AP_Airspeed_ND::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_press_count > 0) {
        _pressure = _press_sum / _press_count;
        _press_count = 0;
        _press_sum = 0;
    }
    bool range_changed = false;
    if(_pressure > 0.85*_current_range_val){ //if above 85% of range, go to the next
        if(_range_setting > 0){
            _range_setting -= 1;
            range_changed = true;
        } // can't go higher
    }else if(_pressure < 0.15*_current_range_val){ //if below 15% of range, go to the next
        if(_range_setting < _available_ranges){
            _range_setting += 1;
            range_changed = true;
        } // can't go lower
    }
    if(range_changed){
        switch(_dev_model){
            case DevModel::ND210:
                _current_range_val = nd210_range[_range_setting];
                break;
            case DevModel::ND005D:
                _current_range_val = nd005d_range[_range_setting];
                break;
            default:
                ::printf("No specific device detected/not supported");
        }
        ::printf("Range changed to %d: %f inH2O\n", _range_setting, _current_range_val);
    }

    pressure = _pressure;
    ::printf("Pressure: %f\t", pressure);
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_ND::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_temp_count > 0) {
        _temperature = _temp_sum / _temp_count;
        _temp_count = 0;
        _temp_sum = 0;
    }

    temperature = _temperature;
    ::printf("Temp: %f\n", temperature);
    return true;
}

#endif  // AP_AIRSPEED_ND_ENABLED
