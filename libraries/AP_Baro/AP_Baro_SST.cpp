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
  backend driver for Superior Sensor's ND015A absolute pressure sensor
 */
#include "AP_Baro_SST.h"

#if AP_BARO_SST_ENABLED

#include <hal.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Common/AP_Common.h>
#include <utility>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

const uint8_t usr_config[2] = {0x29, 0x03}; // 800-1100 range, bw limit set to 100Hz-> 343Hz rate
uint8_t model_sign[7] = {0x56, 0x4E, 0x30, 0x32, 0x36, 0x43, 0x4D};
uint8_t vn_baro_sign[7] = {0x56, 0x4E, 0x2D, 0x42, 0x41, 0x52, 0x4F};

AP_Baro_SST::AP_Baro_SST(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Baro_Backend(baro)
    , dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_SST::probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_SST *sensor = new AP_Baro_SST(baro, std::move(dev));
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_SST::matchModel(uint8_t* reading) {
  for (uint8_t i = 0; i < 7; i++) {
    if (reading[i] != vn_baro_sign[i]) {
      return false;
    }
  }
  return true;
}

void AP_Baro_SST::set_configuration(void)
{
    uint8_t recv[5];
    WITH_SEMAPHORE(dev->get_semaphore());

    // i2cMasterTransmitTimeout(I2CD, (0x2C << 1), usr_config, 2, nullptr, 0, TIME_INFINITE);
    dev->transfer(usr_config, 2, recv, 2);

    // uint8_t buf[5];
	// memcpy(buf, config, 2);
	// bool ret = dev->transfer(&buf[0], 2, nullptr, 2);
    // uint8_t reg = 0x00;
    // uint8_t val = 0;
    // dev->transfer(&reg, 1, &val, 1);
    // return ret;
    // return dev->write_register(0x29, 0x03);
    // return dev->write_register(0x01, config[1]);
}

bool AP_Baro_SST::init()
{
    // dev->get_semaphore()->take_blocking();
    
    // set_configuration();
    
    uint8_t reading[24] = {'\0'};
    uint8_t model[7] = {'\0'};

    dev->get_semaphore()->take_blocking();
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    dev->set_retries(2);
    set_configuration();
    hal.scheduler->delay(10);
    if (!dev->read(reading,24)) {
        dev->get_semaphore()->give();
        return false;
    } else {
        for (int i=0; i < 7; i++) {
            model[i] = reading[i+6];
        }
    }
    
    if (!matchModel(model)) {
        dev->get_semaphore()->give();
        return false;
    }
    hal.scheduler->delay(55);
    
    // if (!set_configuration()) {
    //     dev->get_semaphore()->give();
    //     return false;
    // }

    instance = _frontend.register_sensor();
    dev->set_device_type(DEVTYPE_BARO_SST);
    set_bus_id(instance, dev->get_bus_id());

    // ::printf("Found and matched VN-BARO device.\n");
    
    dev->get_semaphore()->give();
    dev->register_periodic_callback(10000, //2751? 363.5Hz = 1.09kHz/{3} from config[1]
        FUNCTOR_BIND_MEMBER(&AP_Baro_SST::collect, void));

    return true;
}

/*
    convert raw pressure to pressure in psi
*/
float AP_Baro_SST::_get_pressure(uint32_t dp_raw) const
{
    const float psi_to_Pa = 6894.757f;
    const float margin = 5898.24f;
    const float offset = 3276.8f;
    
    float press_psi  = (((float) dp_raw - offset)*1.50)/margin; //fixed 15 psi range for A series
    float press  = press_psi * psi_to_Pa;
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Baro_SST::_get_temperature(int8_t dT_int, int8_t dT_frac) const
{
    float temp  = dT_int + dT_frac/256.0;
    return temp;
}

void AP_Baro_SST::collect()
{
    uint8_t data[6]; //mode, 3 bytes for pressure and 2 for temperature
    WITH_SEMAPHORE(dev->get_semaphore());
    set_configuration();
    if (!dev->read(data, sizeof(data))) {
        return;
    }

    uint32_t dp_raw;
    dp_raw = (data[1] << 16) + (data[2] << 8) + data[3];

    float press  = _get_pressure(dp_raw);
    float temp  = _get_temperature(data[4], data[5]);

    WITH_SEMAPHORE(_sem);
    _press_sum += press ;
    _temp_sum += temp;
    _press_count += 1;
    _temp_count += 1;
    _last_sample_time_ms = AP_HAL::millis();
}

void AP_Baro_SST::update() {
    WITH_SEMAPHORE(_sem);
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100 
        || _press_count <= 0 || _temp_count <= 0) {
        return;
    }

    _pressure = _press_sum / _press_count;
    _temperature = _temp_sum / _temp_count;
    _press_count = 0;
    _press_sum = 0;
    _temp_count = 0;
    _temp_sum = 0;
    _copy_to_frontend(instance, _pressure, _temperature);
}

#endif  // AP_Baro_SST_ENABLED
