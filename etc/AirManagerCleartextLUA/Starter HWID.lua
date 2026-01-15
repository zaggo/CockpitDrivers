-- sim/flightmodel2/engines/starter_is_running -> Starter Light
-- sim/cockpit/engine/ignition_on -> [0] -> Key Position 0 = off 1 = L 2= R 3 = Both

led_starter = hw_output_add("ARDUINO_MEGA2560_B_D34", false)
function starter_light_callback(value)
    hw_output_set(led_starter, value[1] == 1)
end 
xpl_dataref_subscribe("sim/flightmodel2/engines/starter_is_running", "INT[16]", starter_light_callback)

-- This function is called every time the ignition switch has a new position
function switch_ignition_callback(position)
  print("The ignition switch got changed to position " .. position)

  if position == 0 then
    xpl_command("sim/magnetos/magnetos_off_1")
  elseif position == 1 then
    xpl_command("sim/magnetos/magnetos_right_1")
  elseif position == 2 then
     xpl_command("sim/magnetos/magnetos_left_1")
  elseif position == 3 then
    xpl_command("sim/magnetos/magnetos_both_1")
    -- X-Plane requires you to send a begin command (1) to keep the starter engine activated, after that we need to turn it off (0) when going back to BOTH
    xpl_command("sim/starters/engage_starter_1", 0)
  elseif position == 4 then
    -- The starter engine will run as long as you keep it in the last position.
    xpl_command("sim/starters/engage_starter_1", 1)
  end
end

-- Create a 5 position switch
hw_switch_add("ARDUINO_MEGA2560_B_D23", "ARDUINO_MEGA2560_B_D22", "ARDUINO_MEGA2560_B_D24", "ARDUINO_MEGA2560_B_D25", "ARDUINO_MEGA2560_B_D27", switch_ignition_callback)