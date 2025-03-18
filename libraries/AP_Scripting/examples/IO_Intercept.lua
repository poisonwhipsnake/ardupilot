-- ArduPilot Lua script to send packets to ESP32 over Serial

local serial_port = 1  -- Choose the correct serial port (e.g., SERIAL1)
local baud_rate = 115200  -- Must match ESP32 baud rate
local start_byte = 0xAA  -- Start byte for packet format
local time = 0  -- Time variable for sine wave generation
local update_rate = 0.02  -- 50Hz update interval (20ms per update)
local digital_state = 1  -- Initial digital state
local counter = 1

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
    
    gcs:send_text(0, string.format("Sent packet: %02X %02X %02X %02X %02X (Checksum: %02X)", 
        packet[1], packet[2], packet[3], packet[4], packet[5], packet[5]))
end

function update()
    -- Increment digital state and wrap around at 8
    
    counter = counter + 1  -- Increment every 20ms

    if counter >= 50 then  -- Every 1 second (50 × 20ms = 1000ms)
        counter = 0  -- Reset counter
        
        -- Update digital_state every second
        if digital_state == 1 then
            digital_state = 2
        elseif digital_state == 2 then
            digital_state = 4
        elseif digital_state == 4 then
            digital_state = 8
        else
            digital_state = 1
        end

        -- Debugging messages for each state
        if digital_state == 1 then
            gcs:send_text(0, "Park")
        elseif digital_state == 2 then
            gcs:send_text(0, "Neutral")
        elseif digital_state == 4 then
            gcs:send_text(0, "Forward")
        elseif digital_state == 8 then
            gcs:send_text(0, "Reverse")
        end
    end

    
    -- Generate a 1Hz sine wave for analog output (scaled to 12-bit 0-4095)
    local sine_wave = math.sin(2 * math.pi * time)  -- Sine wave from -1 to 1
    local analog_value = math.floor((sine_wave * 0.5 + 0.5) * 4095)  -- Scale to 0-4095
    time = time + update_rate  -- Increment time by update rate (20ms per cycle)
    
    send_packet(digital_state, analog_value)
    return update, 20  -- Repeat every 20ms (50Hz)
end

serial = serial:find_serial(serial_port)
if serial then
    serial:begin(baud_rate)
    gcs:send_text(0, "ArduPilot Lua Serial TX Started")
    return update, 20  -- Start sending packets at 50Hz
else
    gcs:send_text(0, "Failed to open serial port")
end
