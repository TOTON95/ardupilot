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

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SST_ND_ENABLED

/*
  backend driver for Superior Sensor's ND differential pressure sensor
 */

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_SST_ND : public AP_Airspeed_Backend
{
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;
    ~AP_Airspeed_SST_ND(void) {}

    bool init() override;
    bool get_differential_pressure(float &pressure) override;
    bool get_temperature(float &temperature) override;

    static AP_Airspeed_Backend *probe(AP_Airspeed &arspd, int devidx, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    AP_Airspeed_SST_ND(AP_Airspeed &arspd, int devidx,AP_HAL::OwnPtr<AP_HAL::Device> dev);
    enum DevModel : uint8_t
    {
        ND210,
        ND130,
        ND160,
        ND005D,
        SST_ND,
    };

    bool matchModel(uint8_t *reading);
    bool probe(uint8_t bus, uint8_t address);
    void _collect();
    float _get_pressure(uint32_t dp_raw) const;
    float _get_temperature(int8_t dT_int, uint8_t dT_frac) const;
    bool range_change_needed(float last_pressure);
    void change_range(void);

    DevModel _dev_model;

    uint8_t range_ofs;
    uint8_t range_max;
    float current_range_val; // inh2o
    float _temp_sum;
    float _press_sum;
    uint32_t _temp_count;
    uint32_t _press_count;
    float _temperature;
    float _pressure;
    uint32_t _last_sample_time_ms;
    bool need_setup = false;
    uint8_t instance;
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
};

#endif // AP_AIRSPEED_SST_ND_ENABLED