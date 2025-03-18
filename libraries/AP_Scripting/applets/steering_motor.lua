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
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TEST', 5.7), 'could not add param2')

local gain = Parameter("LUA_STR_GAIN")
local param2 = Parameter("LUA_STR_TEST")





--------------------------------------------------------------------------------------------------
-- ArduPilot Lua Script for controlling RS232 Motor
-- Connects to a motor controller via Serial Port (RS232)

local serial_port = 0  -- Set to the correct serial port (SERIAL2 for example)
local baud_rate = 115200
local motor_enabled = false
local max_speed = 10000 -- Max speed in the manual

function setup_serial()
    -- Configure the serial port
    serial = serial:find_serial(serial_port)
    if serial then
        serial:begin(baud_rate)
        serial:set_flow_control(0)
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
    gcs:send_text(0, "Motor Disabled")
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

    -- Convert throttle (-1 to 1) to motor speed (-10000 to 10000)
    local pos = math.floor(steering_demand * max_speed)

    -- Convert speed to 4-byte representation
    local speed_bytes = int_to_bytes(pos)

    -- Construct the full command
    local cmd = {0xE0, 0x01, 0x00, 0x00, speed_bytes[1], speed_bytes[2], speed_bytes[3], speed_bytes[4]}

    send_bytes(cmd)


    -- gcs:send_text(0, "Sent Speed: " .. pos)
end

function update()
    
    local steering_demand = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    send_pos_command(steering_demand)
    return update, 20 -- Run this function every 100ms
end

setup_serial()
gcs:send_text(0, "Steering Motor Lua Running ")

return update()

