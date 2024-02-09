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
#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SST_ND_ENABLED

#include "AP_Airspeed_SST_ND.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
//#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define ND_I2C_ADDR1 0x28
#define ND_I2C_ADDR2 0x30

const float inH20_to_Pa = 249.08f;

const float LOW_RANGE_LVL = 0.25;
const float HIGH_RANGE_LVL = 0.80;

const float nd210_range[7] = {10.0, 5.0, 4.0, 2.0, 1.0, 0.5, 0.25}; // all in inH2O
const float nd130_range[6] = {30.0, 20.0, 10.0, 5.0, 4.0, 2.0};
const float nd160_range[8] = {60.0, 50.0, 40.0, 30.0, 20.0, 10.0, 5.0, 2.5};
const float nd005d_range[6] = {138.4, 110.72, 55.36, 27.68, 22.14, 13.84}; // converted psi to inH2O
const float vn131cm_range[8] = {51.2, 47.2, 43.3, 39.4, 35.4, 31.5, 27.5, 23.6};

uint8_t config_setting[2] = {0x54, 0x00}; // notch filter disabled, bw limit set to 50Hz-> 148Hz odr with auto select, wdg disabled, pressure range set to 0b100
uint8_t sst_config_setting[2] = {0x0A, 0x07}; //bw limit set to 50Hz -> 155.35Hz, pressure range set to 0b010

const float *range;

AP_Airspeed_SST_ND::AP_Airspeed_SST_ND(AP_Airspeed &arspd, int devidx, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Airspeed_Backend(arspd, devidx)
    , dev(std::move(_dev))
{
}

// probe for a sensor
bool AP_Airspeed_SST_ND::probe(uint8_t bus, uint8_t address)
{
   /* _dev = hal.i2c_mgr->get_device(bus, address);
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(20);
    uint8_t reading[14];

    if(!_dev->read(reading, 14)){
        return false;
    }
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Found bus %u addr 0x%02x", _dev->bus_num(), _dev->get_bus_address());
    return matchModel(&reading[6]);*/
    return false;
}

AP_Airspeed_Backend *AP_Airspeed_SST_ND::probe(AP_Airspeed &arspd, int devidx, AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Airspeed_SST_ND *sensor = new AP_Airspeed_SST_ND(arspd, devidx, std::move(dev));
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// probe and initialise the sensor
bool AP_Airspeed_SST_ND::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(10);
    uint8_t reading[14];
    if (!dev->read(reading, sizeof(reading))) {
        return false;
    }
    
    if (!matchModel(&reading[6])) {
        return false;
    }

    //TODO: Clean code

    dev->set_device_type(uint8_t(DevType::SST_ND));
    set_bus_id(dev->get_bus_id());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO,"SST_ND[%u]: Found bus %u addr 0x%02x", get_instance(), _dev->bus_num(), _dev->get_bus_address());

    switch (_dev_model)
    {
    case DevModel::SST_ND:
        range_max = ARRAY_SIZE(vn131cm_range);
        range = vn131cm_range;
        range_ofs = 7;
        break;
    case DevModel::ND210:
        range_max = ARRAY_SIZE(nd210_range);
        range = nd210_range;
        range_ofs = 6;
        break;
    case DevModel::ND005D:
        range_max = ARRAY_SIZE(nd005d_range);
        range = nd005d_range;
        range_ofs = 6;
        break;
    case DevModel::ND160:
        range_max = ARRAY_SIZE(nd160_range);
        range = nd160_range;
        range_ofs = 7;
        break;
    case DevModel::ND130:
        range_max = ARRAY_SIZE(nd130_range);
        range = nd130_range;
        range_ofs = 6;
        break;
    }

    // Set current range value
    current_range_val = range[range_ofs];

    // drop to 10 retries for runtime
    dev->set_retries(10);

    // Setup a variable for initial configuration
    need_setup = true;

    dev->register_periodic_callback(20000, //  6757 for 148Hz ODR
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_SST_ND::_collect, void));
    return true;
}

bool AP_Airspeed_SST_ND::matchModel(uint8_t* model)
{ 
    static const struct {
        const char *str;
        DevModel model;
    } models[] = {
        { "VN131CM", DevModel::SST_ND },
        { "ND210", DevModel::ND210 },
        { "ND130", DevModel::ND130 },
        { "ND160", DevModel::ND160 },
        { "ND005D", DevModel::ND005D },
    };
    for (const auto &s : models) {
        if (strcmp(s.str, (const char*) model)) {
            continue;
        }
        _dev_model = s.model;
        return true;
    }
    return false;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_SST_ND::_get_temperature(int8_t dT_int, uint8_t dT_frac) const
{
    return dT_int + dT_frac/256.0;
}

// read the values from the sensor
void AP_Airspeed_SST_ND::_collect()
{
    uint8_t data[6]; //1 byte for mode, 3 bytes for pressure and 2 for temperature
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!dev->read(data, sizeof(data))) {
        return;
    }

    // Setup initial configuration
    if (need_setup) {
        dev->transfer(sst_config_setting, 2, nullptr,0);
        need_setup = false;
    }

    const uint32_t dp_raw { (uint32_t)(data[1] << 16) | (data[2] << 8) | data[3] };

    float press  = _get_pressure(dp_raw);
    float temp  = _get_temperature(data[4], data[5]);

    WITH_SEMAPHORE(sem);

    _press_sum += press;
    _temp_sum += temp;
    _press_count += 1;
    _temp_count += 1;

    _last_sample_time_ms = AP_HAL::millis();
}

/*
    convert raw pressure to pressure in Pascals
*/
float AP_Airspeed_SST_ND::_get_pressure(uint32_t dp_raw) const
{
    const float margin = 29491.2f;
    float diff_press_inH2O = 0.0f;

    if (_dev_model != DevModel::SST_ND) {
        diff_press_inH2O = (dp_raw * current_range_val) / margin;
    } else {
        diff_press_inH2O = (float)(current_range_val *
                                    (dp_raw - 8388607.5f) /
                                    15099493.5f);
    }
    return diff_press_inH2O * inH20_to_Pa;
}

bool AP_Airspeed_SST_ND::range_change_needed(float last_pressure)
{
    // if above 80% of range, go to the next
    if (last_pressure > HIGH_RANGE_LVL*current_range_val*inH20_to_Pa) { 
        if (range_ofs > 0) {
            range_ofs -= 1;
            return true;
        }
        return false;
    }

    // if below 25% of range, go to the next
    if (last_pressure < LOW_RANGE_LVL*current_range_val*inH20_to_Pa) { 
        if (range_ofs < range_max - 1) {
            range_ofs += 1;
            return true;
        } 
    }
    return false;
}

// set the range of the sensor
void AP_Airspeed_SST_ND::change_range()
{
    //Update range
    current_range_val = range[range_ofs];   
    config_setting[0] = (config_setting[0] & 0xF8) + (0b0111 - range_ofs);
    WITH_SEMAPHORE(dev->get_semaphore());
    //_dev->transfer(config_setting, 2, nullptr,0);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Range changed to %d: %.2f inH2O\n", range_ofs, current_range_val);
}

// get the differential pressure in Pascals
bool AP_Airspeed_SST_ND::get_differential_pressure(float &pressure)
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
    
    if (range_change_needed(_pressure)) {
        change_range();     
    }
    pressure = _pressure;
    return true;
}

bool AP_Airspeed_SST_ND::get_temperature(float &temperature)
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
    return true;
}

#endif  // AP_AIRSPEED_SST_ND_ENABLED