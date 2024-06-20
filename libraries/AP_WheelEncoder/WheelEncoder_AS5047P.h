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

    // constructor
    using AP_WheelEncoder_Backend::AP_WheelEncoder_Backend;
    // update state
    void update(float unused, float unused2) override;

private:
    uint64_t last_update_ms = 0;
    uint16_t last_encoder_value = 0;
    float last_angle = 0.0f;

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
};
