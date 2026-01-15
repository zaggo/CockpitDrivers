-- Motor constants
kTurnrate = 1
kSideslip = 2
kAirspeed = 3
kVerticalSpeed = 4
kEngineSpeedRPM = 5
kAttitudeIndicator = 7

function new_message(id, payload)
    --print("Received " .. id)
end

cockpit_driver = hw_message_port_add("ARDUINO_MEGA2560_P", new_message)

busVoltageOk = false
busVoltageInitialized = false
vacuumOk = false
vacuumInitialized = false

attitudeScaledRoll = 0
attitudeScaledPitch = 0
scaledTurnrate = 999
scaledSideslip = 999
scaledAirspeed = 999
scaledRpm = 999
scaledVerticalSpeed = 999


-- Gyro ---
function attitudeIndicatorPitch_callback(deg)
    -- value between 0.0 ... 1.0, 0.5 = 0° 0.2=-90° 0.78=+90°
    local newPitch = var_cap(var_round(deg, 1), -30, 30)
    if not vacuumOk then
        newPitch = 30
    end
    if newPitch ~= attitudeScaledPitch then
        attitudeScaledPitch = newPitch
        -- print("AttitudePitch: "..deg.." attitudeScaledRoll: "..attitudeScaledRoll.." attitudeScaledPitch: "..attitudeScaledPitch)
        hw_message_port_send(cockpit_driver, kAttitudeIndicator, "FLOAT[2]", { attitudeScaledRoll, attitudeScaledPitch })
    end
end

xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot", "FLOAT", attitudeIndicatorPitch_callback)

function attitudeIndicatorRoll_callback(deg)
    -- value between 0.0 ... 1.0, 0.5 = 0° 0.2=-90° 0.78=+90°
    local newRoll = -var_round(deg, 1)
    if not vacuumOk then
        newRoll = -30
    end
    if newRoll ~= attitudeScaledRoll then
        attitudeScaledRoll = newRoll
        -- print("AttitudeRoll: "..deg.." attitudeScaledRoll: "..attitudeScaledRoll.." attitudeScaledPitch: "..attitudeScaledPitch)
        hw_message_port_send(cockpit_driver, kAttitudeIndicator, "FLOAT[2]", { attitudeScaledRoll, attitudeScaledPitch })
    end
end

xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot", "FLOAT", attitudeIndicatorRoll_callback)

function turnrate_callback(deg)
    -- value between 0.0 ... 1.0, 0.5 = 0° 0.2=-90° 0.78=+90°
    local clippedDeg = var_cap(var_round(deg, 1), -110, 110)
    local newTurnrate = 0.5 - (clippedDeg / 90) * 0.29
    if not busVoltageOk then
        newTurnrate = 0
    end
    if newTurnrate ~= scaledTurnrate then
        scaledTurnrate = newTurnrate
        -- print("Turnrate: "..deg.." clipped: "..clippedDeg.." scaledTurnrate = "..scaledTurnrate)
        hw_message_port_send(cockpit_driver, kTurnrate, "FLOAT", scaledTurnrate)
    end
end

xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot", "FLOAT", turnrate_callback)


function sideslip_callback(deg)
    -- value between 0.0 ... 0.156  0° = 0.082
    -- deg between -8 (full right) and 8 (full left)
    local zero = 0.084
    local data = {
        { -1, 0 },
        { 0,  zero },
        { 1,  0.156 }
    }

    local sideslip = 0
    local normalizedDeg = -var_round(deg, 2) / 8.0

    if not busVoltageOk then
        sideslip = zero
    elseif normalizedDeg <= data[1][1] then
        sideslip = data[1][2]
    elseif normalizedDeg >= data[#data][1] then
        sideslip = data[#data][2]
    else
        for i = 1, #data - 1 do
            local k1, v1 = data[i][1], data[i][2]
            local k2, v2 = data[i + 1][1], data[i + 1][2]

            if normalizedDeg >= k1 and normalizedDeg <= k2 then
                -- lin Interpolation
                sideslip = v1 + (v2 - v1) * (normalizedDeg - k1) / (k2 - k1)
                break
            end
        end
    end
    if sideslip ~= scaledSideslip then
        scaledSideslip = sideslip
        print("Sliprate: " .. deg .. " scaledSideslip = " .. scaledSideslip)
        hw_message_port_send(cockpit_driver, kSideslip, "FLOAT", scaledSideslip)
    end
end

xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/slip_deg", "FLOAT", sideslip_callback)


function airspeed_callback(knots)
    local data = {
        { 0,   0 },
        { 40,  0.0900 },
        { 50,  0.1340 },
        { 60,  0.2000 },
        { 70,  0.2660 },
        { 80,  0.3410 },
        { 90,  0.4400 },
        { 100, 0.5300 },
        { 110, 0.6200 },
        { 120, 0.7000 },
        { 130, 0.7700 },
        { 140, 0.8450 },
        { 150, 0.8990 },
        { 160, 0.9600 },
        { 167, 1.0000 }
    }

    local airspeed = 0
    local clippedKnots = var_cap(var_round(knots, 1), 0, 167)

    if not busVoltageOk then
        airspeed = 0
    elseif clippedKnots <= data[1][1] then
        airspeed = data[1][2]
    elseif clippedKnots >= data[#data][1] then
        airspeed = data[#data][2]
    else
        for i = 1, #data - 1 do
            local k1, v1 = data[i][1], data[i][2]
            local k2, v2 = data[i + 1][1], data[i + 1][2]

            if clippedKnots >= k1 and clippedKnots <= k2 then
                -- lin Interpolation
                airspeed = v1 + (v2 - v1) * (clippedKnots - k1) / (k2 - k1)
                break
            end
        end
    end

    if airspeed ~= scaledAirspeed then
        scaledAirspeed = airspeed
        print("Airspeed: "..knots.." scaledAirspeed = "..scaledAirspeed)
        hw_message_port_send(cockpit_driver, kAirspeed, "FLOAT", 1 - scaledAirspeed)
    end
end

xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", "FLOAT", airspeed_callback)

function verticalspeed_callback(fpm)
    local data = {
        { -1810, 0 },
        { -1500, 0.908 },
        { -1000, 0.755 },
        { -500,  0.613 },
        { 0,     0.5 },
        { 500,   0.391 },
        { 1000,  0.245 },
        { 1500,  0.085 },
        { 1810,  0 }
    }

    local verticalSpeed = 0
    local clippedFpm = var_cap(var_round(fpm, 0), -1810, 1810)

    if not busVoltageOk then
        verticalSpeed = 0
    elseif clippedFpm <= data[1][1] then
        verticalSpeed = data[1][2]
    elseif clippedFpm >= data[#data][1] then
        verticalSpeed = data[#data][2]
    else
        for i = 1, #data - 1 do
            local k1, v1 = data[i][1], data[i][2]
            local k2, v2 = data[i + 1][1], data[i + 1][2]

            if clippedFpm >= k1 and clippedFpm <= k2 then
                -- Lineare Interpolation
                verticalSpeed = v1 + (v2 - v1) * (clippedFpm - k1) / (k2 - k1)
                break
            end
        end
    end

    if verticalSpeed ~= scaledVerticalSpeed then
        scaledVerticalSpeed = verticalSpeed
        -- print("Vertical Speed: "..fpm.." scaledVerticalSpeed = "..scaledVerticalSpeed)
        hw_message_port_send(cockpit_driver, kVerticalSpeed, "FLOAT", scaledVerticalSpeed)
    end
end

xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/vvi_fpm_pilot", "FLOAT", verticalspeed_callback)

function engine_speed_rpm_callback(speed)
    local data = {
        { 0,    0.100 },
        { 500,  0.222 },
        { 1000, 0.346 },
        { 1500, 0.467 },
        { 2000, 0.59 },
        { 2500, 0.714 },
        { 3000, 0.83 },
        { 3500, 0.95 }
    }

    local rpm = 0
    local clippedRpm = var_cap(var_round(speed[1], 1), 0, 3500)

    if not busVoltageOk then
        rpm = 0
    elseif clippedRpm <= data[1][1] then
        rpm = data[1][2]
    elseif clippedRpm >= data[#data][1] then
        rpm = data[#data][2]
    else
        for i = 1, #data - 1 do
            local k1, v1 = data[i][1], data[i][2]
            local k2, v2 = data[i + 1][1], data[i + 1][2]

            if clippedRpm >= k1 and clippedRpm <= k2 then
                -- lin Interpolation
                rpm = v1 + (v2 - v1) * (clippedRpm - k1) / (k2 - k1)
                break
            end
        end
    end

    if rpm ~= scaledRpm then
        scaledRpm = rpm
        --print("Raw: "..speed[1].." RPM: "..rpm.." scaledRpm = "..scaledRpm)
        hw_message_port_send(cockpit_driver, kEngineSpeedRPM, "FLOAT", 1 - scaledRpm)
    end
end

xpl_dataref_subscribe("sim/cockpit2/engine/indicators/engine_speed_rpm", "FLOAT[16]", engine_speed_rpm_callback)

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
        request_callback(sideslip_callback)
        request_callback(turnrate_callback)
        request_callback(verticalspeed_callback)
    end
end

xpl_dataref_subscribe("sim/cockpit2/electrical/bus_volts", "FLOAT[6]", bus_volts_callback)

function vacuum_callback(value)
    local vacuum = value
    local reloadNeeded = false
    if vacuum < 0.1 then
        reloadNeeded = vacuumOk or (not vacuumInitialized)
        vacuumOk = false
    else
        reloadNeeded = not vacuumOk or (not vacuumInitialized)
        vacuumOk = true
    end

    if reloadNeeded then
        vacuumInitialized = true
        print("vacuum = " .. vacuum .. " ok = " .. tostring(vacuumOk))
        request_callback(attitudeIndicatorRoll_callback)
        request_callback(attitudeIndicatorPitch_callback)
    end
end

xpl_dataref_subscribe("sim/cockpit/misc/vacuum", "FLOAT", vacuum_callback)
