led_stall = hw_output_add("ARDUINO_MEGA2560_B_D31", false)
led_gear_up = hw_output_add("ARDUINO_MEGA2560_B_D28", false)
led_gear_trans = hw_output_add("ARDUINO_MEGA2560_B_D29", false)
led_aas = hw_output_add("ARDUINO_MEGA2560_B_D26", false)

led_nose_gear = hw_output_add("ARDUINO_MEGA2560_B_D32", false)
led_left_gear = hw_output_add("ARDUINO_MEGA2560_B_D30", false)
led_right_gear = hw_output_add("ARDUINO_MEGA2560_B_D33", false)

local busVoltageOk = false
local busVoltageInitialized = false

function test_annunc_callback(typecmd)
    -- Can be BEGIN, ONCE or END
   if typecmd ~= "END" then
     print("test_annunc_callback "..typecmd.." busVoltageOK="..tostring(busVoltageOk))
     hw_output_set(led_stall, busVoltageOk)
     hw_output_set(led_gear_up, busVoltageOk)
     hw_output_set(led_gear_trans, busVoltageOk)
     hw_output_set(led_aas, busVoltageOk)
   else -- if typecmd == "END" then
     hw_output_set(led_stall, false)
     hw_output_set(led_gear_up, false)
     hw_output_set(led_gear_trans, false)
     hw_output_set(led_aas, false)
   end
end
xpl_command_subscribe("sim/annunciator/test_all_annunciators", test_annunc_callback)

function stall_callback(value)
    print("stall_callback "..value.." busVoltageOK="..tostring(busVoltageOk))
    if busVoltageOk then
        hw_output_set(led_stall, value == 1)
    else
        hw_output_set(led_stall, false)
    end
end 
xpl_dataref_subscribe("sim/cockpit2/annunciators/stall_warning", "INT", stall_callback)

function gear_up_callback(value)
    if busVoltageOk then
        hw_output_set(led_gear_up, value == 1)
    else
        hw_output_set(led_gear_up, false)
    end
end 
xpl_dataref_subscribe("sim/cockpit2/annunciators/gear_warning", "INT", gear_up_callback)

function gear_trans_callback(value)
    if busVoltageOk then
        hw_output_set(led_gear_trans, value == 1)
    else
        hw_output_set(led_gear_trans, false)
    end
end 
xpl_dataref_subscribe("sim/cockpit/warnings/annunciators/gear_unsafe", "INT", gear_trans_callback)

function aas_callback(value)
    if busVoltageOk then
        hw_output_set(led_aas, value == 1)
    else
        hw_output_set(led_aas, false)
    end
end 
xpl_dataref_subscribe("VFLYTEAIR/ARROWIII/AASSwitch", "INT", aas_callback)

function noseGear_callback(value)
    if busVoltageOk then
        hw_output_set(led_nose_gear, value == 1)
    else
        hw_output_set(led_nose_gear, false)
    end
end 
xpl_dataref_subscribe("sim/flightmodel/movingparts/gear1def", "FLOAT", noseGear_callback)
function leftGear_callback(value)
    if busVoltageOk then
        hw_output_set(led_left_gear, value == 1)
    else
        hw_output_set(led_left_gear, false)
    end
end 
xpl_dataref_subscribe("sim/flightmodel/movingparts/gear2def", "FLOAT", leftGear_callback)
function rightGear_callback(value)
    print("rightGear_callback "..value.." busVoltageOK="..tostring(busVoltageOk))
    if busVoltageOk then
        hw_output_set(led_right_gear, value == 1)
    else
        hw_output_set(led_right_gear, false)
    end
end 
xpl_dataref_subscribe("sim/flightmodel/movingparts/gear3def", "FLOAT", rightGear_callback)

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
        request_callback(rightGear_callback)  
        request_callback(leftGear_callback) 
        request_callback(noseGear_callback)
         
        request_callback(stall_callback) 
        request_callback(gear_up_callback) 
        request_callback(aas_callback) 
        request_callback(gear_trans_callback)
         
        request_callback(test_annunc_callback)          
    end
end
xpl_dataref_subscribe("sim/cockpit2/electrical/bus_volts", "FLOAT[6]", bus_volts_callback)
