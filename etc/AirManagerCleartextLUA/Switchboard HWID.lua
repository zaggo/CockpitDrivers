-- PushButton relations
local switchConfigurations = {
    {hardware_id = "ARDUINO_MEGA2560_A_D5", sim_ref = "sim/electrical/generator_1"},
    {hardware_id = "ARDUINO_MEGA2560_A_D7", sim_ref = "sim/electrical/battery_1"},
    {hardware_id = "ARDUINO_MEGA2560_A_D8", sim_ref = "sim/fuel/fuel_pump_1"},
    {hardware_id = "ARDUINO_MEGA2560_A_D9", sim_ref = "sim/lights/landing_lights"},
    {hardware_id = "ARDUINO_MEGA2560_A_D10", sim_ref = "sim/lights/nav_lights"},
    {hardware_id = "ARDUINO_MEGA2560_A_D11", sim_ref = "sim/lights/beacon_lights"},
    {hardware_id = "ARDUINO_MEGA2560_A_D12", sim_ref = "sim/lights/strobe_lights"},
    {hardware_id = "ARDUINO_MEGA2560_A_D13", sim_ref = "sim/lights/taxi_lights"},
    {hardware_id = "ARDUINO_MEGA2560_A_A4", sim_ref = "sim/systems/avionics"},
}

function createSwitchHandler(simRef)
    return function(position)
        if position > 0 then
            xpl_command(simRef.."_on", "ONCE")
        else
            xpl_command(simRef.."_off", "ONCE")
        end
    end
end

print("Setup Switchboard…")
print("Installing switch handlers")
for _, config in ipairs(switchConfigurations) do
    local handler = createSwitchHandler(config.sim_ref)
     hw_switch_add(config.hardware_id, handler)
end

print("Installing pitot heat switch handler")
function switch_pitot_heat_callback(position)
    print("pitot heat is "..position)
    if position > 0 then
        xpl_command("sim/ice/pitot_heat0_on", "ONCE")
        xpl_command("sim/ice/pitot_heat_2_on", "ONCE")
    else
        xpl_command("sim/ice/pitot_heat0_off", "ONCE")
        xpl_command("sim/ice/pitot_heat_2_off", "ONCE")
    end
end
hw_switch_add("ARDUINO_MEGA2560_A_A3", switch_pitot_heat_callback)

print("Installing dome light switch handler")
function switch_dome_light_callback(position)
    print("Dome Light is "..position)
    if position > 0 then
        xpl_dataref_write("sim/cockpit2/switches/panel_brightness_ratio", "FLOAT[4]", {1.0}, 1)
    else
        xpl_dataref_write("sim/cockpit2/switches/panel_brightness_ratio", "FLOAT[4]", {0.0}, 1)
    end
end
hw_switch_add("ARDUINO_MEGA2560_A_A5", switch_dome_light_callback)

print("Installing parking break handler")
function switch_parkingbrake_callback(position)
    -- print("Parking Brake is "..position)
    if position == 0 then
        xpl_dataref_write("sim/cockpit2/controls/parking_brake_ratio", "FLOAT", 1)
    else
        xpl_dataref_write("sim/cockpit2/controls/parking_brake_ratio", "FLOAT", 0.0)
    end
end
hw_switch_add("ARDUINO_MEGA2560_A_D4", switch_parkingbrake_callback)

print("Setup ready!")
