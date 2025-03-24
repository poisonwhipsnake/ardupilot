-- Rover motor driver for an Ackerman style vehicle (i.e. the frame has separate throttle and steering controls)
--
-- The following parameters should be set:
--     SERVO1_FUNCTION = 94 (Script 1) 
--     SERVO3_FUNCTION = 96 (Script 3)
-- 
-- The Frame's steering control should be connected to the autopilot's output1, throttle control to output3
--
-- CAUTION: This script only works for Rover
-- This script retrieves the high level controller outputs that have been sent to the regular motor driver
-- and then outputs them to the "Script 1" and "Script 3" outputs.  This does not add any real value beyond
-- serving as an example of how lua scripts can be used to implement a custom motor driver 


local CONTROL_OUTPUT_YAW = 4

local PARAM_TABLE_KEY = 150

assert(param:add_table(PARAM_TABLE_KEY, "LUA_STR_", 3), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'GAIN', 10000), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'MAX', 45.0), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'WB', 4.50), 'could not add param2')

local gain = Parameter("LUA_STR_GAIN")
local max_steering_angle = Parameter("LUA_STR_MAX")
local wheel_base = Parameter("LUA_STR_WB")

local encoder_query_cmd = {0xED, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  -- Query encoder position
local last_position_request_time = 0  -- Timestamp for requesting position

--------------------------------------------------------------------------------------------------
-- ArduPilot Lua Script for controlling RS232 Motor
-- Connects to a motor controller via Serial Port (RS232)

local serial_port = 0  -- Set to the correct serial port (SERIAL2 for example)
local baud_rate = 115200
local motor_enabled = false
local max_speed = 10000 -- Max speed in the manual
local previous_yaw = 0

local filtered_angle_demand = 0
local filtered_angle_estimate= 0

local gain_estimate = 0
local wheel_angle_estimate = 0
local centre_estimate = 0

-- Buffer for handling serial data
local serial_buffer = {}  

function setup_serial()
    -- Configure the serial port
    serial = serial:find_serial(serial_port)
    if serial then
        serial:begin(baud_rate)
  
        serial:set_flow_control(0)  -- Disable hardware flow control

        gcs:send_text(0, "Serial port initialized")
    else
        gcs:send_text(0, "Serial port not found")
    end
end

function enable_motor()
    local cmd = {0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    send_bytes(cmd)
    motor_enabled = true
    gcs:send_text(0, "Motor Enabled")
end

function disable_motor()
    local cmd = {0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    send_bytes(cmd)
    motor_enabled = false
end

function int_to_bytes(value)
    -- Convert signed 32-bit integer to 4 bytes (big-endian)
    local b1 = (value >> 24) & 0xFF -- Highest byte
    local b2 = (value >> 16) & 0xFF
    local b3 = (value >> 8) & 0xFF
    local b4 = value & 0xFF -- Lowest byte
    return {b1, b2, b3, b4}
end

function send_bytes(cmd)
    -- Ensure each byte is sent as an integer
    for i = 1, #cmd do
        serial:write(tonumber(cmd[i])) -- Convert each byte to an integer
    end
end

function send_pos_command(steering_demand)
    if not motor_enabled then
        enable_motor()
    end

    -- Convert steering (-1 to 1) to motor position range
    local pos = math.floor(-steering_demand * gain:get())

    pos = pos + math.floor(centre_estimate)

    -- Convert to 4-byte representation
    local speed_bytes = int_to_bytes(pos)

    local cmd = {0xE0, 0x01, 0x00, 0x00, speed_bytes[1], speed_bytes[2], speed_bytes[3], speed_bytes[4]}
    send_bytes(cmd)
end

function request_encoder_position()
    send_bytes(encoder_query_cmd)
end

function check_serial_buffer()
    local max_bytes_per_cycle = 8  -- Limit processing to avoid timeouts
    local processed_bytes = 0

    while serial:available() > 0 and processed_bytes < max_bytes_per_cycle do
        local byte = serial:read()  -- Read one byte at a time
        table.insert(serial_buffer, byte)
        processed_bytes = processed_bytes + 1

        -- Keep buffer size limited to 8 bytes (only store latest message)
        if #serial_buffer > 8 then
            table.remove(serial_buffer, 1)  -- Remove oldest byte
        end

        -- Process if we have a full 8-byte message
        if #serial_buffer == 8 then
            -- Print raw buffer content for debugging
            
            if serial_buffer[1] ~= 0x00 then
                local buffer_str = "Serial Buffer: "
                for i = 1, #serial_buffer do
                    buffer_str = buffer_str .. string.format("%02X ", serial_buffer[i])
                end
                --gcs:send_text(0, buffer_str)  -- Send raw buffer contents to GCS

                -- Check if it's an encoder position message
                if serial_buffer[1] == 0xED and serial_buffer[2] == 0x08 then
                    -- Extract the encoder position
                    local position = (serial_buffer[5] << 24) | (serial_buffer[6] << 16) |
                                    (serial_buffer[7] << 8) | serial_buffer[8]
                    last_position = position
                    gcs:send_text(0, "Current Encoder Position: " .. position)
                end
                 -- Clear buffer after processing message
                serial_buffer = {}
            end
            
           
        end
    end
end

function update_virtual_angle_sensor()
    -- Update the virtual angle sensor
    local current_speed = ahrs:groundspeed_vector():length()
    if current_speed < 0.1 then
        previous_yaw = ahrs:get_yaw()
        return
    end

    if gain_estimate == 0 then
        gain_estimate = gain:get()
    end
    
    local steering_demand = vehicle:get_control_output(CONTROL_OUTPUT_YAW)

    local yaw_change = ahrs:get_yaw() - previous_yaw
    if yaw_change > math.pi then
        yaw_change = yaw_change - (2 * math.pi)
    elseif yaw_change < -math.pi then
        yaw_change = yaw_change + (2 * math.pi)
    end
    previous_yaw = ahrs:get_yaw()
   
    local yaw_rate = yaw_change / 0.02 --radians per seconds
    local curvature = yaw_rate / current_speed
    local steering_angle_current_estimate = math.atan(wheel_base:get() * curvature) * 180 / math.pi
    local steering_angle_current_demand = steering_demand * max_steering_angle:get()
    filtered_angle_demand = (filtered_angle_demand*0.98) + (steering_angle_current_demand *0.02)
    filtered_angle_estimate = (filtered_angle_estimate*0.98) + (steering_angle_current_estimate *0.02)


    local gain_adjust = 1
    if ((math.abs(filtered_angle_demand)>1.0) and (math.abs(filtered_angle_estimate)>1.0))then
        
        gain_adjust = filtered_angle_demand / filtered_angle_estimate
        if gain_adjust < 0 then
            gain_adjust = 1
        end
        gain_adjust = ((gain_adjust-1)*0.001)+1
    end

    local angle_error = filtered_angle_estimate - filtered_angle_demand
    local offset_adjust = angle_error*0.1
    if angle_error < 0 then
        offset_adjust = -offset_adjust
    end

    gain_estimate = gain_estimate * gain_adjust
    centre_estimate = centre_estimate + offset_adjust

    --if one second has elapsed print the values
    local now = millis()
    if (now - last_position_request_time) > 1000 then
        last_position_request_time = now
        gcs:send_text(2, "Steering Angle Estimate: " .. filtered_angle_estimate)
        gcs:send_text(2, "Steering Angle Demand: " .. filtered_angle_demand)
        gcs:send_text(2, "Gain Estimate: " .. gain_estimate)
        gcs:send_text(2, "Centre Estimate: " .. centre_estimate)
    end


    
    
end

function update()
    local armed = arming:is_armed()

    if armed then
        local steering_demand = vehicle:get_control_output(4)
        send_pos_command(steering_demand)
        update_virtual_angle_sensor()
    else
        disable_motor()
    end
    
    -- Check if we received an encoder response (or discard other messages)
    check_serial_buffer()

    return update, 20 -- Run every 100ms
end

setup_serial()
gcs:send_text(0, "Steering Motor Lua Running")

return update()
