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

#include "WheelEncoder_AS5047P.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


void AP_WheelEncoder_AS5047P::update(float unused, float unused2)
{
    if (!_dev) {
        _dev = std::move(hal.spi->get_device("as5047p"));
        _dev->get_semaphore()->take_blocking();
        _dev->set_retries(10);
        gcs().send_text(MAV_SEVERITY_INFO, "Wheel encoder setup");
        _dev->get_semaphore()->give();
    }
    else {
        // read the position of the encoder
        uint16_t address = 0x3FFF;
        uint16_t rw = 1; // 1: read, 0: write
        
        uint16_t data = address  | (rw<<14);

        // if data is odd, add parity bit
        uint16_t parity_data = data;
        bool parity = true;
        while(parity_data) {
            parity = !parity;
            parity_data = parity_data & (parity_data - 1);
        }
        if(parity) {
            data = data ;
        }else{
            data = data  | 1 <<15;
        }
    
        uint8_t buf[2];
        buf[0] = (data >> 8) & 0xFF;
        buf[1] = data & 0xFF;

        _dev->get_semaphore()->take_blocking();

        uint8_t reply[2];
        _dev->transfer(buf, 2, reply, 0);

        hal.scheduler->delay_microseconds(5);

        _dev->transfer(buf, 0, reply, 2);

        uint16_t encoder_value = (reply[0] << 8) | reply[1];

        encoder_value = encoder_value & 0x3FFF;

        _dev->get_semaphore()->give();


        // calculate the angle
        float angle = encoder_value * 360.0f / 16384.0f;

        //gcs().send_text(MAV_SEVERITY_INFO, "Encoder Angle: %f", angle);

        // calculate the change in angle
        float delta_angle = angle - last_angle;

        // check for wrap around
        if (delta_angle > 180.0f) {
            delta_angle -= 360.0f;
        } else if (delta_angle < -180.0f) {
            delta_angle += 360.0f;
        }

        // update the state
        _state.raw_angle = angle;

        // save the last angle
        last_angle = angle;


    }

}

