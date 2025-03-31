-- Safety Manager Script



gpio:pinMode(51,1) -- set servo 10 to output, gpio:pinMode(51,0) would be input

gpio:pinMode(50,0) -- set servo 9 to input, gpio:pinMode(51,0) would be input

auth_id = arming:get_aux_auth_id()

tractor_estop_armed = false

function update()
  
  -- if input changes state, send_tex

  local tractor_estop_status = gpio:read(50)

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
  
  if arming:is_armed() then
    gpio:write(51, 0)
  else
    gpio:write(51, 1)
  end

  return update, 1000
end

return update() -- run immediately before starting to reschedule
