local busVoltageOk = false
local busVoltageInitialized = false

------------------------
-- Instrument Brightness
------------------------

-- Potentionmeter -> Xplane
function panel_brightness_knob_callback(value)
  print("panel brightness knob: "..value)
  xpl_dataref_write("sim/cockpit/electrical/instrument_brightness", "FLOAT", value)
  -- sim/cockpit2/switches/instrument_brightness_ratio (32 0/1)
end
panel_brightness_id = hw_adc_input_add("ARDUINO_MEGA2560_B_A0", panel_brightness_knob_callback)
panel_brightness_knob_callback(hw_adc_input_read(panel_brightness_id))

-- Potentionmeter -> Xplane
function radio_brightness_knob_callback(value)
  print("Radio brightness knob: "..value)
  xpl_dataref_write("sim/cockpit2/switches/instrument_brightness_ratio", "FLOAT[32]", {value}, 1)
end
radio_brightness_id = hw_adc_input_add("ARDUINO_MEGA2560_B_A1", radio_brightness_knob_callback)
radio_brightness_knob_callback(hw_adc_input_read(radio_brightness_id))

-----------
-- Switches
-----------

function switch_aas_callback(position)
    print("AAS is "..position)
    xpl_dataref_write("VFLYTEAIR/ARROWIII/AASSwitch", "INT", position)
end
hw_switch_add("ARDUINO_MEGA2560_B_D35", switch_aas_callback)

function switch_annunc_callback(position)
    print("Annunc is "..position)
    if position == 1 then
        xpl_command("sim/annunciator/test_all_annunciators", "BEGIN")
    else
        xpl_command("sim/annunciator/test_all_annunciators", "END")
    end
end
hw_switch_add("ARDUINO_MEGA2560_B_D36", switch_annunc_callback)

---------
-- LIGHTS
---------
panel_brightness_pwm = hw_output_pwm_add("ARDUINO_MEGA2560_B_D13", 1000, 0.5)
dummy1 = hw_output_pwm_add("ARDUINO_MEGA2560_B_D11", 1000, 0.5)
dummy2 = hw_output_pwm_add("ARDUINO_MEGA2560_B_D12", 1000, 0.5)
function panel_brightness_callback(value)
    local brightness = value
    if not busVoltageOk then
        brightness = 0
    end
    print("panel_brightness_pwm: " .. value)
    hw_output_pwm_duty_cycle(panel_brightness_pwm, brightness)
end
xpl_dataref_subscribe("sim/cockpit/electrical/instrument_brightness", "FLOAT", panel_brightness_callback)

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
        request_callback(panel_brightness_callback)     
    end
end
xpl_dataref_subscribe("sim/cockpit2/electrical/bus_volts", "FLOAT[6]", bus_volts_callback)

request_callback(panel_brightness_callback)     
print("Setup DimmerPanel ready")