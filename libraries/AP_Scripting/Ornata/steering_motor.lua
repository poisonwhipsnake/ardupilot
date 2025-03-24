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

local PARAM_TABLE_KEY = 72

assert(param:add_table(PARAM_TABLE_KEY, "LUA_STR_", 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'GAIN', 10000), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'MAX', 45.0), 'could not add param2')

local gain = Parameter("LUA_STR_GAIN")
local param2 = Parameter("LUA_STR_TEST")

local encoder_query_cmd = {0xED, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  -- Query encoder position
local last_position_request_time = 0  -- Timestamp for requesting position
local last_position = nil  -- Store last known position

--------------------------------------------------------------------------------------------------
-- ArduPilot Lua Script for controlling RS232 Motor
-- Connects to a motor controller via Serial Port (RS232)

local serial_port = 0  -- Set to the correct serial port (SERIAL2 for example)
local baud_rate = 115200
local motor_enabled = false
local max_speed = 10000 -- Max speed in the manual


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


function update()
    local armed = arming:is_armed()

    if armed then
        local steering_demand = vehicle:get_control_output(4)
        send_pos_command(steering_demand)
        update_virtual_angle_sensor()
    else
        disable_motor()
    end

    -- Request encoder position every second
    local now = millis()
    if (now - last_position_request_time) > 1000 then
        last_position_request_time = now
        --request_encoder_position()
    end
    
    -- Check if we received an encoder response (or discard other messages)
    check_serial_buffer()

    return update, 20 -- Run every 100ms
end

setup_serial()
gcs:send_text(0, "Steering Motor Lua Running")

return update()
