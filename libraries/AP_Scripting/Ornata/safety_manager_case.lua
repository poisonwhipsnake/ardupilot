-- Safety Manager Script



gpio:pinMode(58,1) -- set servo 10 to output, gpio:pinMode(51,0) would be input

gpio:pinMode(59,1) -- set servo 10 to output, gpio:pinMode(51,0) would be input

gpio:pinMode(57,0) -- set servo 9 to input, gpio:pinMode(51,0) would be input

auth_id = arming:get_aux_auth_id()

tractor_estop_armed = false

safety_armed = false

safety_off_time =0

safety_delay_ms = 3000

function update()
  
  -- if input changes state, send_tex
---------------------------------------------
  local tractor_estop_status = gpio:read(57)

  if tractor_estop_status and (not tractor_estop_armed) then
    gcs:send_text(0, "Safety Manager: Tractor E-Stop Armed")
  elseif (not tractor_estop_status) and tractor_estop_armed then
    gcs:send_text(0, "Safety Manager: Tractor E-Stop Disarmed")
  end

  tractor_estop_armed = tractor_estop_status

  if tractor_estop_armed then
    arming:set_aux_auth_passed(auth_id)
  else
    arming:set_aux_auth_failed(auth_id, "Tractor ESTOP Not Armed")
  end
---------------------------------------------
  
  local current_time = millis()

  if arming:is_armed() then
    if current_time - safety_off_time > safety_delay_ms then
      if not safety_armed then
        gcs:send_text(0, "Safety Manager: Safety Armed")
      end
      safety_armed = true
      gpio:write(58, 0)
      gpio:write(59, 0)
    end
  else
    safety_off_time = current_time
    safety_armed = false
    gpio:write(58, 1)
    gpio:write(59, 1)
  end

  return update, 20
end

return update() -- run immediately before starting to reschedule
