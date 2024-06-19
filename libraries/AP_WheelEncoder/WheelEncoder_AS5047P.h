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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "WheelEncoder_Backend.h"
#include <AP_Math/AP_Math.h>

class AP_WheelEncoder_AS5047P : public AP_WheelEncoder_Backend
{
public:
    static AP_WheelEncoder_Backend *probe(AP_WheelEncoder &encoder,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);

    // update state
    void update(void) override;

private:
    int32_t  _distance_count; // distance count as number of encoder ticks
    uint32_t _total_count; // total number of encoder ticks

    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    bool write_register(uint8_t reg, uint8_t v);

    void check_err_reg();

    bool hardware_init();

    bool init();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;
};
