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
#include "AP_Airspeed_SST.h"

#if AP_AIRSPEED_SST_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define ND_I2C_ADDR1 0x28
#define ND_I2C_ADDR2 0x30

uint16_t print_counter=0;

uint8_t config_reg[2] = {0x54, 0x00}; // notch filter disabled, bw limit set to 50Hz-> 148Hz odr with auto select, wdg disabled, pressure range set to 0b100
uint8_t configVN_reg [2] = {0x29, 0x01};
uint8_t MN_ND210[8] = {0x4E, 0x44, 0x32, 0x31, 0x30, 0x00, 0x00, 0x00};
uint8_t MN_VN131CM[8] = {0x56, 0x4E, 0x31, 0x33, 0x31, 0x43, 0x4D, 0x00};


float nd210_range[7] = {10.0, 5.0, 4.0, 2.0, 1.0, 0.5, 0.25}; // all in inH2O
float nd130_range[6] = {30.0, 20.0, 10.0, 5.0, 4.0, 2.0};
float vn131_range[8] = {130.0, 120.0, 110.0, 100.0, 90.0, 80.0, 70.0, 60.0}; //converted cmH2O to in H2O

AP_Airspeed_SST::AP_Airspeed_SST(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

bool AP_Airspeed_SST::matchModel(uint8_t* reading) 
{
    for (int i = 0; i < 8; i++) { 
        if(reading[i] != MN_VN131CM[i]) {
            goto probeND210;
        }
        _dev_model = DevModel::VN131;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"VN131CM dev type detected.\n");
        return true;
    }
    probeND210:
    for (int i = 0; i < 8; i++) {
        if (reading[i] != MN_ND210[i]) {
            goto probeND005;
        }
        _dev_model = DevModel::ND210;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"ND210 dev type detected.\n");
        return true;
    }
    probeND005:
    for (int i = 0; i < 8; i++) {
        if (reading[i] != MN_ND005D[i]) {
            return false;
        }
        _dev_model = DevModel::ND005D;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ND005D dev type detected.\n");
    }
  return true;
}

// probe for a sensor
bool AP_Airspeed_SST::probe(uint8_t bus, uint8_t address)
{
    _dev = hal.i2c_mgr->get_device(bus, address);
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(5);
    uint8_t reading[14]= {'\0'};
    uint8_t model[8] = {'\0'};
    if(!_dev->read(reading, 14)){
        return false;
    }else{
        for (int i=6; i<14; i++){
                model[i-6]= reading[i];
        }
    }
    return matchModel(model);
}

// probe and initialise the sensor
bool AP_Airspeed_SST::init()
{
    static const uint8_t addresses[] = { ND_I2C_ADDR1 };
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
    return false;

found_sensor:
    _dev->set_device_type(uint8_t(DevType::SSTx));
    set_bus_id(_dev->get_bus_id());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"SST_ND[%u]: Found bus %u addr 0x%02x", get_instance(), _dev->bus_num(), _dev->get_bus_address());

    // send default configuration
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer(config_reg, 2, nullptr,0);


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
        case DevModel::VN131:
            _available_ranges = 8;
            _range_setting = 3;
            _current_range_val = vn131_range[_range_setting];
            break;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"ND not setup correctly");
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(6757,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_SST::_collect, void));
    return true;
}

/*
    convert raw pressure to pressure in Pascals
*/
float AP_Airspeed_SST::_get_pressure(int32_t dp_raw) const
{
    const float inH20_to_Pa = 249.08f;
    const float cmH20_to_Pa = 98.06f;
    const float margin = 29491.2f;
    const float margin24b = 7549746.3f;
    float press = -0.1010;

    if(_dev_model == DevModel::VN131) {
        float diff_press_cmH2O = _current_range_val*((dp_raw - 8388607.5f)/margin);
        press = diff_press_cmH2O * cmH20_to_Pa;
    } else {
        float diff_press_inH2O  = (_current_range_val*dp_raw)/margin;
        press  = diff_press_inH2O * inH20_to_Pa;
    }
    
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_SST::_get_temperature(int8_t dT_int, int8_t dT_frac) const
{
    float temp  = dT_int + dT_frac/256.0;
    return temp;
}

// read the values from the sensor
void AP_Airspeed_SST::_collect()
{
    uint8_t data[5]; //3 bytes for pressure and 2 for temperature

    if (!_dev->read(data, sizeof(data))) {
        return;
    }

    int32_t dp_raw;
    dp_raw = (data << 16) + (data[2] << 8) + data[3];

    float press  = _get_pressure(dp_raw);
    float temp  = _get_temperature(data[4], data[5]);

    WITH_SEMAPHORE(sem);

    _press_sum += press ;
    _temp_sum += temp;
    _press_count += 1;
    _temp_count += 1;

    _last_sample_time_ms = AP_HAL::millis();
}

bool AP_Airspeed_SST::get_differential_pressure(float &pressure)
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
    const float inH20_to_Pa = 249.08f;
    if(_pressure > 0.8*_current_range_val*inH20_to_Pa){ //if above 85% of range, go to the next
        if(_range_setting > 0){
            _range_setting -= 1;
            range_changed = true;
        } // can't go higher
    }else if(_pressure < 0.25*_current_range_val*inH20_to_Pa){ //if below 15% of range, go to the next
        if(_range_setting < _available_ranges - 1){
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
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"No specific device detected/not supported\n");
        }
        config_reg[0] = (config_reg[0] & 0xF0) + (0b0111 - _range_setting);
        WITH_SEMAPHORE(_dev->get_semaphore());
        _dev->transfer(config_reg, 2, nullptr,0);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Range changed to %d: %.2f inH2O\n", _range_setting, _current_range_val);
        hal.scheduler->delay(2); // wait for the sensor to change range
    }
    pressure = _pressure;
    return true;
}

bool AP_Airspeed_SST::get_temperature(float &temperature)
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

#endif  // AP_AIRSPEED_SST_ENABLED
