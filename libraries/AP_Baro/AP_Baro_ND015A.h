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

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
$
#ifndef AP_BARO_ND015A_ENABLED
#define AP_BARO_ND015A_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#if AP_BARO_ND015A_ENABLED

/*
  backend driver for Superior Sensor's ND015A absolute pressure sensor
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HALgit add/I2CDevice.h>
#include <utility>

#include "AP_Baro_Backend.h"

class AP_Baro_ND015A : public AP_Baro_Backend
{
public:
    AP_Baro_ND015A(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
    ~AP_Baro_ND015A(void) {}
    
   /* AP_Baro public interface: */
    void update() override;

    // bool probe(uint8_t bus, uint8_t address);
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    bool matchModel(uint8_t* reading);
    float _get_pressure(int16_t dp_raw) const;
    float _get_temperature(int8_t dT_int, int8_t dT_frac) const;

    float _temp_sum;
    float _press_sum;
    uint32_t _temp_count;
    uint32_t _press_count;
    float _temperature;
    float _pressure;
    uint32_t _last_sample_time_ms;

    
};

#endif  // AP_BARO_ND015A_ENABLED