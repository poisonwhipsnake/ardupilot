-- ArduPilot Lua script to send packets to ESP32 over Serial

local serial_port = 1  -- Choose the correct serial port (e.g., SERIAL1)
local baud_rate = 115200  -- Must match ESP32 baud rate
local poll_time = 20
local start_byte = 0xAA  -- Start byte for packet format
local CONTROL_OUTPUT_THROTTLE = 3

local PARAM_TABLE_KEY = 200

assert(param:add_table(PARAM_TABLE_KEY, "IO_", 7), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'CLUTCH_T', 3000), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'STOP_SPEED', 0.1), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'STOP_TIME', 1000), 'could not add param3')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'MIN_THR', 0.1), 'could not add param4')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'CLUTCH_WAIT', 1000), 'could not add param5')
assert(param:add_param(PARAM_TABLE_KEY, 7, 'CLUTCH_MAX', 3500), 'could not add param6')
--assert(param:add_param(PARAM_TABLE_KEY, 8, 'CHANGE_TIME', 500), 'could not add param7')

local Clutch_Travel_Time = Parameter("IO_CLUTCH_T")
local Stop_Speed = Parameter("IO_STOP_SPEED")
local Stop_Time = Parameter("IO_STOP_TIME")
local Min_Throttle = Parameter("IO_MIN_THR")
local Clutch_Wait_Time = Parameter("IO_CLUTCH_WAIT")
local Clutch_Max_PWM = Parameter("IO_CLUTCH_MAX")
local Change_Time = 500 --Parameter("IO_CHANGE_TIME")


-- Moving Status - 0 = Stopped, 1 = Forward, -1 = Reverse
local moving = 0
-- Clutch Status - 0 = Clutch Out + Neutral/Park, 1 = Clutch in + Neutral/Park, 2 = Clutch In +Forward/Reverse, 3 = Clutch Out + Forward/Reverse
local clutch_status = 0

-- loops since clutch started moving
local clutch_counter = 0

local stopped_counter = 0

local change_counter = 0

-- Forward, Neutral, Reverse, Park State Machine, 1 = Park, 2 = Neutral, 4 = Forward, 8 = Reverse
local FNRP_State = 1

local clutch_output = Clutch_Max_PWM:get()

local previous_FNRP_state = FNRP_State



-- Manual bitwise operations for Lua 5.4 (if 'bit' or 'bit32' is unavailable)
function bxor(a, b)
    local result = 0
    local bitval = 1
    while a > 0 or b > 0 do
        local abit, bbit = a % 2, b % 2
        if abit ~= bbit then
            result = result + bitval
        end
        a, b, bitval = math.floor(a / 2), math.floor(b / 2), bitval * 2
    end
    return result
end

function band(a, b)
    local result = 0
    local bitval = 1
    while a > 0 and b > 0 do
        if (a % 2 == 1) and (b % 2 == 1) then
            result = result + bitval
        end
        a, b, bitval = math.floor(a / 2), math.floor(b / 2), bitval * 2
    end
    return result
end

function rshift(x, by)
    return math.floor(x / (2 ^ by))
end

function calculate_checksum(data)
    local checksum = 0
    for i = 1, #data do  -- Compute checksum over all bytes except itself
        checksum = bxor(checksum, data[i])
    end
    return checksum
end

function send_packet(digital_state, analog_value)
    local packet = {}
    packet[1] = start_byte
    packet[2] = digital_state
    packet[3] = rshift(analog_value, 8)  -- High byte of 12-bit analog value
    packet[4] = band(analog_value, 0xFF) -- Low byte of 12-bit analog value
    
    -- Compute checksum before adding to packet
    local checksum = calculate_checksum({packet[1], packet[2], packet[3], packet[4]})
    packet[5] = checksum  
    
    -- Send each byte individually as a number
    for i = 1, #packet do
        serial:write(packet[i])
    end
    
    --gcs:send_text(0, string.format("Sent packet: %02X %02X %02X %02X %02X (Checksum: %02X)", 
    --   packet[1], packet[2], packet[3], packet[4], packet[5], packet[5]))
end

function update()
    -- Increment digital state and wrap around at 8
    
    
    --local throttleDemand = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)
    local throttleDemand = (rc:get_pwm(3)-1500)/400
    --gcs:send_text(0, tostring(throttleDemand))
    local ground_speed = ahrs:groundspeed_vector():length()
    --gcs:send_text(0, tostring(ground_speed))
    local armed = arming:is_armed()
    local go = false
    local FNRP_Target_State = 2

    if not armed then
        moving = 0
        clutch_status = 0
        clutch_counter = 0
        FNRP_State = 1
        clutch_output = Clutch_Max_PWM:get()

    else 
        if throttleDemand > Min_Throttle:get() and moving > -1 then
            moving = 1
            go = true
            FNRP_Target_State = 4
        elseif throttleDemand < -Min_Throttle:get() and moving < 1 then
            moving = -1
            go = true
            FNRP_Target_State = 8
        else
            FNRP_State = 2
            clutch_output = 0
            clutch_counter = 0
            clutch_status = 0
        end

        if go then
            stopped_counter = 0
            if clutch_status == 0 then
                clutch_status = 1
                clutch_counter = 0
                FNRP_State = 2
                clutch_output = 0
            
            elseif clutch_status == 1 then
                if clutch_counter < (Clutch_Wait_Time:get()/poll_time) then
                    clutch_counter = clutch_counter + 1
                    FNRP_State = 2
                    clutch_output = 0
                else
                    FNRP_State = FNRP_Target_State
                    clutch_output = 0
                    clutch_status = 2
                    clutch_counter = 0
                end
            elseif clutch_status == 2 then
                if clutch_counter < (Clutch_Travel_Time:get()/poll_time) then 
                    clutch_counter = clutch_counter + 1
                    clutch_output =  math.floor((Clutch_Max_PWM:get()*clutch_counter*poll_time)/Clutch_Travel_Time:get())
                    FNRP_State = FNRP_Target_State
                else
                    clutch_status = 3
                    clutch_output = Clutch_Max_PWM:get()
                    FNRP_State = FNRP_Target_State
                end
            else
                clutch_output = Clutch_Max_PWM:get()
                FNRP_State = FNRP_Target_State
            end
        else
            if (ground_speed<Stop_Speed:get()) and moving ~= 0  then
                stopped_counter = stopped_counter + 1
            else
                stopped_counter = 0
            end
        
            if stopped_counter > (Stop_Time:get()/poll_time) and moving ~= 0 then
                moving = 0
                gcs:send_text(0, "Lua Stop Detected")
            end  
        end
    
    end

    if FNRP_State ~= previous_FNRP_state then

        change_counter = 0;
        if FNRP_State == 1 then
            gcs:send_text(0, "Park")
        elseif FNRP_State == 2 then
            gcs:send_text(0, "Neutral")
        elseif FNRP_State == 4 then
            gcs:send_text(0, "Forward")
        elseif FNRP_State == 8 then
            gcs:send_text(0, "Reverse")
        end
    end

    local FNRP_State_Override = FNRP_State

    if (change_counter < (Change_Time /poll_time)) then
        change_counter = change_counter + 1
        FNRP_State_Override = 0;
    end

    previous_FNRP_state = FNRP_State
    
    send_packet(FNRP_State_Override, clutch_output)

    return update, poll_time  -- Repeat every 20ms (50Hz)
end

serial = serial:find_serial(serial_port)
if serial then
    serial:begin(baud_rate)
    gcs:send_text(0, "ArduPilot Lua Serial TX Started")
    return update, 20  -- Start sending packets at 50Hz
else
    gcs:send_text(0, "Failed to open serial port")
end
