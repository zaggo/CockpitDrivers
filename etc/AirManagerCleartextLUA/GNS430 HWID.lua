local busVoltageOk = false
local busVoltageInitialized = false

-- PushButton relations
local pushButtonConfigurations = {
    {hardware_id = "ARDUINO_MEGA2560_A_D23", sim_ref = "sim/GPS/g430n1_vvol"},
    {hardware_id = "ARDUINO_MEGA2560_A_D26", sim_ref = "sim/GPS/g430n1_nav_com_tog"},
    {hardware_id = "ARDUINO_MEGA2560_A_D31", sim_ref = "sim/GPS/g430n1_com_ff"},
    {hardware_id = "ARDUINO_MEGA2560_A_D32", sim_ref = "sim/GPS/g430n1_nav_ff"},
    {hardware_id = "ARDUINO_MEGA2560_A_D33", sim_ref = "sim/GPS/g430n1_cdi"},
    {hardware_id = "ARDUINO_MEGA2560_A_D34", sim_ref = "sim/GPS/g430n1_obs"},
    {hardware_id = "ARDUINO_MEGA2560_A_D35", sim_ref = "sim/GPS/g430n1_msg"},
    {hardware_id = "ARDUINO_MEGA2560_A_D36", sim_ref = "sim/GPS/g430n1_fpl"},
    {hardware_id = "ARDUINO_MEGA2560_A_D37", sim_ref = "sim/GPS/g430n1_proc"},
    {hardware_id = "ARDUINO_MEGA2560_A_D38", sim_ref = "sim/GPS/g430n1_zoom_in"},
    {hardware_id = "ARDUINO_MEGA2560_A_D39", sim_ref = "sim/GPS/g430n1_zoom_out"},
    {hardware_id = "ARDUINO_MEGA2560_A_D40", sim_ref = "sim/GPS/g430n1_direct"},
    {hardware_id = "ARDUINO_MEGA2560_A_D41", sim_ref = "sim/GPS/g430n1_menu"},
    {hardware_id = "ARDUINO_MEGA2560_A_D42", sim_ref = "sim/GPS/g430n1_clr"},
    {hardware_id = "ARDUINO_MEGA2560_A_D43", sim_ref = "sim/GPS/g430n1_ent"},
    {hardware_id = "ARDUINO_MEGA2560_A_D44", sim_ref = "sim/GPS/g430n1_cursor"}
}

local rotaryEncoderConfigurations = {
    {hardware_id_A = "ARDUINO_MEGA2560_A_D45", hardware_id_B = "ARDUINO_MEGA2560_A_D46", sim_ref_up = "sim/GPS/g430n1_page_up", sim_ref_dn = "sim/GPS/g430n1_page_dn"},
    {hardware_id_A = "ARDUINO_MEGA2560_A_D47", hardware_id_B = "ARDUINO_MEGA2560_A_D48", sim_ref_up = "sim/GPS/g430n1_chapter_up", sim_ref_dn = "sim/GPS/g430n1_chapter_dn"},
    {hardware_id_A = "ARDUINO_MEGA2560_A_D29", hardware_id_B = "ARDUINO_MEGA2560_A_D30", sim_ref_up = "sim/GPS/g430n1_fine_up", sim_ref_dn = "sim/GPS/g430n1_fine_down"},
    {hardware_id_A = "ARDUINO_MEGA2560_A_D28", hardware_id_B = "ARDUINO_MEGA2560_A_D27", sim_ref_up = "sim/GPS/g430n1_coarse_up", sim_ref_dn = "sim/GPS/g430n1_coarse_down"},
    {hardware_id_A = "ARDUINO_MEGA2560_A_D22", hardware_id_B = "ARDUINO_MEGA2560_A_D50", sim_ref_up = "sim/GPS/g430n1_cvol_up", sim_ref_dn = "sim/GPS/g430n1_cvol_dn"},
    {hardware_id_A = "ARDUINO_MEGA2560_A_D24", hardware_id_B = "ARDUINO_MEGA2560_A_D25", sim_ref_up = "sim/GPS/g430n1_vvol_up", sim_ref_dn = "sim/GPS/g430n1_vvol_dn"}
}

function createPushButtonHandlers(simRef)
    local function pressed()
        print("down "..simRef)
        xpl_command(simRef, "BEGIN")
    end

    local function released()
        print("up "..simRef)
        xpl_command(simRef, "END")
    end

    return pressed, released
end

function createRotaryEncoderHandler(simRefUp, simRefDn)
    return function(direction)
        if direction > 0 then
            xpl_command(simRefUp, "ONCE")
        elseif direction < 0 then
            xpl_command(simRefDn, "ONCE")
        end
    end
end

print("Setup GNS430 Buttons & Encoders")
print("Installing push button handlers")
for _, config in ipairs(pushButtonConfigurations) do
    local pressed, released = createPushButtonHandlers(config.sim_ref)
    hw_button_add(config.hardware_id, pressed, released)
end
print("Installing rotary encoder handlers")
for _, config in ipairs(rotaryEncoderConfigurations) do
    local encoder_change = createRotaryEncoderHandler(config.sim_ref_up, config.sim_ref_dn)
    hw_dial_add(config.hardware_id_A, config.hardware_id_B, encoder_change)
end

print("CVol push button with long press handler")
-- Special case cvol: Long press will shutdown the Raspberry
local lastPressedId = 0
function cvol_pressed()
    lastPressedId = lastPressedId + 1  
    local currentId = lastPressedId
    xpl_command("sim/GPS/g430n1_cvol", "BEGIN")

    timer_start(2000, function()
        if currentId == lastPressedId then
            print("Shutdown")
            shut_down("APPLICATION")
        end
    end)
end

function cvol_released()
    lastPressedId = lastPressedId + 1  
    xpl_command("sim/GPS/g430n1_cvol", "END")
end
hw_button_add("ARDUINO_MEGA2560_A_D49", cvol_pressed, cvol_released)

print("Radio Backlight Handler")
-- Xplane -> PWM Pin Out
radio_brightness_pwm = hw_output_pwm_add("ARDUINO_MEGA2560_A_D6", 1000, 0.5)
function radio_brightness_changed_callback(value)
    local brightness = value[2]
    if not busVoltageOk then
        brightness = 0
    end
    --print("radio_brightness_pwm: " .. value)
    hw_output_pwm_duty_cycle(radio_brightness_pwm, brightness)
end
xpl_dataref_subscribe("sim/cockpit2/switches/instrument_brightness_ratio", "FLOAT[32]", radio_brightness_changed_callback)

--------------
-- Bus Voltage
--------------
print("Subscribing to Bus Voltage")
function bus_volts_callback(volts)
    local busVolts = volts[1]
    local reloadNeeded = false
    if busVolts<7 then
        reloadNeeded = busVoltageOk or (not busVoltageInitialized)
        busVoltageOk = false
    else
        reloadNeeded = not busVoltageOk or (not busVoltageInitialized)
        busVoltageOk = true
    end
    
    if reloadNeeded then
        busVoltageInitialized = true
        print("busVoltage = "..busVolts.." ok = "..tostring(busVoltageOk))
        request_callback(radio_brightness_changed_callback)     
    end
end
xpl_dataref_subscribe("sim/cockpit2/electrical/bus_volts", "FLOAT[6]", bus_volts_callback)

print("Setup ready")