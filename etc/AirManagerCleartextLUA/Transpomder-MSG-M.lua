kTransponderBrightness = 1
kTransponderIdent = 2
kTransponderCode = 3
kTransponderMode = 4
kTransponderLight = 5

function new_message(id, payload)
    --print("Received " .. id)
    if id == kTransponderCode then
        -- Convert string to integer
        local codeInt = tonumber(payload)
        print("Transponder Ident received: " .. codeInt)
        xpl_dataref_write("sim/cockpit2/radios/actuators/transponder_code", "INT", codeInt)
    end
    if id == kTransponderMode then
        local modeInt = tonumber(payload)
        print("Transponder Mode received: " .. modeInt)
        --xpl_dataref_write("sim/cockpit/radios/transponder_mode", "INT", modeInt)
        xpl_dataref_write("VFLYTEAIR/AXP340/AXP340_MODE", "INT", modeInt)
        
        
    end
    if id == kTransponderIdent then
        print("Transponder Ident requested")
        xpl_command("sim/transponder/transponder_ident", "ONCE")
    end
end

cockpit_driver = hw_message_port_add("ARDUINO_NANO_M", new_message)

busVoltageOk = false
busVoltageInitialized = false

-- Transponder ---
function transpondercode_callback(code)
    -- Convert Integer code to String squawkCode, padding leading 0s, if necessary
    local squawkCode = string.format("%04d", code)
    print("XPTransponder Code: " .. squawkCode)
    hw_message_port_send(cockpit_driver, kTransponderCode, "STRING", squawkCode)
end

xpl_dataref_subscribe("sim/cockpit2/radios/actuators/transponder_code", "INT", transpondercode_callback)

function transpondermode_callback(mode)
    -- If busvoltage not ok, set mode to 'off'
    if busVoltageOk ~= true then
        mode = 0
    end
    print("XPTransponder Mode: " .. mode)
    hw_message_port_send(cockpit_driver, kTransponderMode, "INT", mode)
end

xpl_dataref_subscribe("sim/cockpit2/radios/actuators/transponder_mode", "INT", transpondermode_callback)

--[[ function transponderbrightness_callback(brightness)
    -- value between 0.0 ... 1.0
    local clippedBrightness = var_cap(var_round(brightness, 2), 0.0, 1.0)
    print("Transponder Brightness: "..clippedBrightness)
    hw_message_port_send(cockpit_driver, kTransponderBrightness, "FLOAT", clippedBrightness)
end
xpl_dataref_subscribe("sim/cockpit2/radios/indicators/transponder_brightness", "FLOAT", transponderbrightness_callback) ]]

function transponderid_callback(ident)
    print("Transponder Ident: " .. tostring(ident))
    hw_message_port_send(cockpit_driver, kTransponderIdent, "INT", ident)
end
xpl_dataref_subscribe("sim/cockpit2/radios/indicators/transponder_id", "INT", transponderid_callback)

function transponderlight_callback(on)
    local lightOn = (on ~= 0)
    -- print("Transponder Light: " .. tostring(lightOn))
    hw_message_port_send(cockpit_driver, kTransponderLight, "INT", lightOn and 1 or 0)
end
xpl_dataref_subscribe("sim/cockpit/radios/transponder_light", "INT", transponderlight_callback)

function bus_volts_callback(volts)
    local busVolts = volts[1]
    local reloadNeeded = false
    if busVolts < 7 then
        reloadNeeded = busVoltageOk or (not busVoltageInitialized)
        busVoltageOk = false
    else
        reloadNeeded = not busVoltageOk or (not busVoltageInitialized)
        busVoltageOk = true
    end

    if reloadNeeded then
        busVoltageInitialized = true
        print("busVoltage = " .. busVolts .. " ok = " .. tostring(busVoltageOk))
        request_callback(transpondermode_callback)
    end
end

xpl_dataref_subscribe("sim/cockpit2/electrical/bus_volts", "FLOAT[6]", bus_volts_callback)
