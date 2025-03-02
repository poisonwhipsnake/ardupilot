-- This script is an example of reading from the CAN bus

-- Load CAN driver1. The first will attach to a protocol of 10, the 2nd to a protocol of 12
-- this allows the script to distinguish packets on two CAN interfaces
local driver1 = CAN:get_device(25)
local driver2 = CAN:get_device2(5)

if not driver1 and not driver2 then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

-- Only accept DroneCAN node status msg on second driver
-- node status is message ID 341
-- Message ID is 16 bits left shifted by 8 in the CAN frame ID.
driver2:add_filter(uint32_t(0xFFFF) << 8, uint32_t(341) << 8)

local object_count = 0
local current_object_index = 0

function show_frame(dnum, frame)
    if frame and frame:id() then
        gcs:send_text(0,string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
    else
        gcs:send_text(0, "Invalid frame received on CAN[" .. dnum .. "]")
    end
end

function interpret_frame(frame)
    if not frame or not frame:id() then
        gcs:send_text(0, "Invalid frame received")
        return
    end

    local id = tonumber( tostring(frame:id() & 0xF) ) -- Extract last digit to check message type

    if id == 10 then
        -- Message reports number of objects detected
        object_count = frame:data(0)  -- First byte contains object count
        current_object_index = 0
        gcs:send_text(0, "MR72 Detected " .. object_count .. " objects")
    
    elseif id == 11 then
        -- Message contains object distance data
        if current_object_index >= object_count then
            return  -- Avoid processing if more than expected
        end
        current_object_index = current_object_index + 1


        -- Decode object position using MR72 formulas
        local x = (((frame:data(2) & 0x07) * 256) + frame:data(3)) * 0.2 - 204.6
        local y = ((frame:data(1) * 32) + (frame:data(2) >> 3)) * 0.2 - 500
        local distance = math.sqrt(x * x + y * y)
        local dynamic_property = frame:data(6) & 0x07
        local sector = (frame:data(6) >> 3) & 0x03
        local rcs = frame:data(7)

        -- Convert to angle in degrees
        local yaw = math.deg(math.atan(x, y))  



        local origin = ahrs:get_origin()
        local obstacle_loc = ahrs:get_position()

        if obstacle_loc ~= nil then
            obstacle_loc:offset_bearing(yaw + math.deg(ahrs:get_yaw()), distance)

            adsb:set_vehicle_now(current_object_index,current_object_index,obstacle_loc:lat(),obstacle_loc:lng())

        end

        -- Convert to angle in degrees
        local yaw = math.deg(math.atan(x, y))  -- Use atan2 for correct angle calculation

        gcs:send_text(0, string.format("Obj %d: x=%.1fm, y=%.1fm, dp=%i, s=%i, rcs= %i", current_object_index, x, y, dynamic_property, sector, rcs))
    else
        gcs:send_text(0,string.format("CAN[%x] msg from " .. tostring(id) .. ": %X, %i, %i, %i, %i, %i, %i, %i", 10, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
    end
end


function update()
   -- see if we got any frames
   if driver1 then
      local frame = driver1:read_frame()
      if frame then
         -- show_frame(1, frame)
         interpret_frame(frame)
      end
   end
   if driver2 then
      local frame = driver2:read_frame()
      if frame then
         -- show_frame(2, frame)
         interpret_frame(frame)
      end
   end

  return update, 1
end

return update()