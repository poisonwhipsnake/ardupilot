-- ArduPilot Lua script to send packets to ESP32 over Serial

local serial_port = 1  -- Choose the correct serial port (e.g., SERIAL1)
local baud_rate = 115200  -- Must match ESP32 baud rate
local start_byte = 0xAA  -- Start byte for packet format

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
    for i = 1, #data - 1 do  -- Exclude checksum byte
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
    packet[5] = calculate_checksum(packet)  -- Compute checksum before adding it
    
    -- Send each byte individually as a number
    for i = 1, #packet do
        serial:write(packet[i])
    end
    
    gcs:send_text(0, string.format("Sent packet: %02X %02X %02X %02X %02X", 
        packet[1], packet[2], packet[3], packet[4], packet[5]))
end

function update()
    local digital_state = 5  -- Decimal equivalent of 0b00000101 (D27 and D12 HIGH)
    local analog_value = 2048  -- Example: 50% of 12-bit range (0-4095)
    send_packet(digital_state, analog_value)
    return update, 1000  -- Repeat every second
end

serial = serial:find_serial(serial_port)
if serial then
    serial:begin(baud_rate)
    gcs:send_text(0, "ArduPilot Lua Serial TX Started")
    return update, 1000  -- Start sending packets every second
else
    gcs:send_text(0, "Failed to open serial port")
end
