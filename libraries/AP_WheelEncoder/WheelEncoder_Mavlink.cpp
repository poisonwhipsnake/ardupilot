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

#include <AP_HAL/AP_HAL.h>

#include "WheelEncoder_Mavlink.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


void AP_WheelEncoder_Mavlink::update(float wheelAngles[], int8_t relevantInstance){
        last_angle = wheelAngles[relevantInstance];
        // update the state
        _state.wheel_angle = wheelAngles[relevantInstance];
        _state.raw_angle = wheelAngles[relevantInstance];
}
