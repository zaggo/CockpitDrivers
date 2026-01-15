
local forceRefresh

function new_message(id, payload)
  if id == 2 then
    print("Altimete did home")
   -- forceRefresh()
  end
  if id == 3 then
      local payloadMin, inHgMin = 0.342, 28.0
      local payloadMax, inHgMax = 0.626, 32.0
      local inHg = inHgMin + (payload - payloadMin)*(inHgMax-inHgMin)/(payloadMax-payloadMin)
      -- print("Baro received "..payload.." calculated inHg "..inHg)
      xpl_dataref_write("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot", "FLOAT", inHg)
  end
end

alt_driver = hw_message_port_add("ARDUINO_NANO_L", new_message)
hw_message_port_send(alt_driver, 2, "INT", 1)

local altValues = { 0.0 }

function alt_callback(deg)
    local feet = var_round(deg,2)
    if altValues[1] ~= feet then
        altValues[1] = feet
        --print("Altitude: "..deg.." display: "..feet.." raw: "..table.unpack(altValues))
        hw_message_port_send(alt_driver, 1, "FLOAT[1]", {table.unpack(altValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/altitude_ft_pilot", "FLOAT", alt_callback)

forceRefresh = function()
    print("ForceRefresh")
    request_callback(alt_callback)
end
