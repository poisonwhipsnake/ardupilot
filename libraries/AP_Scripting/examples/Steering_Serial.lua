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

local K_SCRIPTING1 = 94 -- for steering control
local K_SCRIPTING3 = 96 -- for throttle control
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4

local port = serial:find_serial(0)

port:begin(57600)
port:set_flow_control(0)



local PARAM_TABLE_KEY = 72

assert(param:add_table(PARAM_TABLE_KEY, "LUA_STR_", 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'GAIN', 10000), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TEST', 5.7), 'could not add param2')

local gain = Parameter("LUA_STR_GAIN")
local param2 = Parameter("LUA_STR_TEST")

function update()

    -- retrieve high level steering and throttle control outputs from vehicle in -1 to +1 range
    local steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)

    port:write(123)

    local gainTemp = gain:get()

    if (steering) then

        local steeringInt = math.floor((steering * gainTemp)+30000)
        local highByte = math.floor(steeringInt / 256) % 256
        local lowbyte = steeringInt % 256
        port:write(highByte)
        port:write(lowbyte)
    else 
        port:write(1)
        port:write(1)
    end

    if not arming:is_armed() then
        port:write(99)
    else
        port:write(88)
    end
    --[[
    if step>100 then
        step = 0
        gcs:send_text(6, "steering script running")
    else
        step = step + 1
    end
    ]]

  return update, 20 -- run at 50hz

end

gcs:send_text(6, "rover-motor-driver.lua is running")
return update()
