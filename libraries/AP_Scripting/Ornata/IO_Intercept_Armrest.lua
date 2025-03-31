-- ArduPilot Lua script to send packets to ESP32 over Serial

local serial_port = 2  -- Choose the correct serial port (e.g., SERIAL1)
local baud_rate = 115200  -- Must match ESP32 baud rate
local poll_time = 20
local start_byte = 0xAA  -- Start byte for packet format


local PARAM_TABLE_KEY = 180

assert(param:add_table(PARAM_TABLE_KEY, "AREST_", 1), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'PTO_ARM', 0), 'could not add param1')

local port = serial:find_serial(serial_port)

if not port then
    gcs:send_text(0, "No Scripting Serial Port")
    return
end

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

--[[
   convert a table of bytes to a lua string
--]]
function bytes_to_string(bytes)
    local ret = {}
    for _, b in ipairs(bytes) do
       if b == 0 then
          break
       end
       table.insert(ret, string.char(b))
    end
    return table.concat(ret)
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
        port:write(packet[i])
    end
    
    --gcs:send_text(0, string.format("Sent packet: %02X %02X %02X %02X %02X (Checksum: %02X)", 
    --   packet[1], packet[2], packet[3], packet[4], packet[5], packet[5]))
end



--[[
   import mavlink support for NAMED_VALUE_INT, only used for
   DUAL_AIRCRAFT operation
--]]
function mavlink_receiver()
    local self = {}
    local mavlink_msgs = require("MAVLink/mavlink_msgs")
    local NAMED_VALUE_INT_msgid = 252 --mavlink_msgs.get_msgid("NAMED_VALUE_INT")
    local msg_map = {}
 
    msg_map[NAMED_VALUE_INT_msgid] = "NAMED_VALUE_INT"
 
    -- initialise mavlink rx with number of messages, and buffer depth
    mavlink.init(1, 10)
 
    -- register message id to receive
    mavlink.register_rx_msgid(NAMED_VALUE_INT_msgid)
 
    --[[
       get a NAMED_VALUE_INT incoming message, handling jitter correction
    --]]
    function self.get_NAMED_VALUE_INT()
       local msg,_,timestamp_ms = mavlink.receive_chan()
       if msg then
          local parsed_msg = mavlink_msgs.decode(msg, msg_map)
          if (parsed_msg ~= nil) and (parsed_msg.msgid == NAMED_VALUE_INT_msgid) then
             -- convert remote timestamp to local timestamp with jitter correction
             local timestamp = timestamp_ms
             local value = parsed_msg.value
             local name = parsed_msg.name--bytes_to_string(parsed_msg.name)
             return timestamp, name, value, parsed_msg.sysid
          end
       end
       return nil
    end
 
    return self
end


mavlink_handler = mavlink_receiver()

last_PTO_request_value = 0
last_remote_timestamp = 0
fresh_message = false
--[[
   handle NAMED_VALUE_INT from another vehicle to sync our schedules
--]]
function handle_PTO_requests()

    local time_boot_ms, name, incoming_value, sysid = mavlink_handler.get_NAMED_VALUE_INT()
    if not time_boot_ms then
        return
    end


    --gcs:send_text(6, string.format("PTO request from %d: %d", name, incoming_value))
    last_PTO_request_value = incoming_value
    last_remote_timestamp = time_boot_ms
    fresh_message = true
    

    --logger.write("PTO",'SysID,PTOR','BI',
    --    sysid, pto_request_value)
    
end


previous_remote_timestamp = 0
previous_update_time = 0

pto_active = false

function update()
    -- Increment digital state and wrap around at 8
    
    local armed = arming:is_armed()
    local current_time = millis()

    handle_PTO_requests()


    if (not fresh_message) and (current_time - previous_update_time) > 1000 and pto_active then
        gcs:send_text(6, string.format("PTO message timeout - disabled"))
        pto_active = false
    end

    if fresh_message then
        fresh_message = false
        previous_update_time = current_time
        gcs:send_text(6, string.format("PTO request %d", last_PTO_request_value))
        if last_PTO_request_value == 123 then
            if armed then
                gcs:send_text(4, "PTO Activated")
                pto_active = true
            else
                gcs:send_text(6, string.format("PTO request %d ignored - not armed", last_PTO_request_value))
                pto_active = false
            end
        else
            pto_active = false
        end
    end

    local pto_output_value = 0
    if pto_active then
        pto_output_value = 1
    end

    send_packet(pto_output_value, 0)

    return update, poll_time  -- Repeat every 20ms (50Hz)

end


if port then
    port:begin(baud_rate)
    gcs:send_text(0, "ArduPilot Lua Serial TX Started")
    return update, 20  -- Start sending packets at 50Hz
else
    gcs:send_text(0, "Failed to open serial port")
end
