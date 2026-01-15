function new_message(id, payload)
  -- Do something with the message from the Arduino
  print("Received "..id)
end

local servo_driver = hw_message_port_add("ARDUINO_NANO_N", new_message)

local servosMsg = 1
local trimServo = 1
local ampsServo = 2
local fuelPressureServo = 3
local egtServo = 4

local normalizedServo = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

-- Common normalization function
local function normalize_value(raw, data)
    if raw <= data[1][1] then
        return data[1][2]
    elseif raw >= data[#data][1] then
        return data[#data][2]
    else
        for i = 1, #data - 1 do
            local k1, v1 = data[i][1], data[i][2]
            local k2, v2 = data[i+1][1], data[i+1][2]
            if raw >= k1 and raw <= k2 then
                return v1 + (v2 - v1) * (raw - k1) / (k2 - k1)
            end
        end
    end
    return 0.0 -- fallback
end

function amps_callback(rawAmps)
    local data = {
        {0, 0},
        {35, 0.37},
        {70, 0.74}
    }
    local amps = rawAmps[1]
    local normalizedAmps = normalize_value(amps, data)
    if normalizedAmps ~= normalizedServo[ampsServo] then
        normalizedServo[ampsServo] = normalizedAmps
        hw_message_port_send(servo_driver, servosMsg, "FLOAT[8]", {table.unpack(normalizedServo)})
    end
end

xpl_dataref_subscribe("sim/cockpit2/electrical/generator_amps", "FLOAT[8]", amps_callback)

function fuelpress_callback(rawPress)
    local data = {
        {0, 0},
        {2, 0.2},  
        {12, 0.68},
        {15, 0.74}
    }
    local pressure = rawPress[1]
    local normalizedPressure = normalize_value(pressure, data)
    if normalizedPressure ~= normalizedServo[fuelPressureServo] then
        normalizedServo[fuelPressureServo] = normalizedPressure
        hw_message_port_send(servo_driver, servosMsg, "FLOAT[8]", {table.unpack(normalizedServo)})
    end
end

xpl_dataref_subscribe("sim/cockpit2/engine/indicators/fuel_pressure_psi", "FLOAT[8]", fuelpress_callback)

function egt_callback(rawDeg)
    local data = {
        {415, 0},
        {605, 0.598},
        {700, 0.88}
    }
    local deg = rawDeg[1]
    local normalizedDeg = normalize_value(deg, data)
    if normalizedDeg ~= normalizedServo[egtServo] then
        normalizedServo[egtServo] = normalizedDeg
        hw_message_port_send(servo_driver, servosMsg, "FLOAT[8]", {table.unpack(normalizedServo)})
    end
end
xpl_dataref_subscribe("sim/cockpit2/engine/indicators/EGT_deg_cel", "FLOAT[16]", egt_callback)

function elevatorTrim_callback(rawTrim)
    local data = {
        {-1, 0.05},
        {-0.659, 0.165},
        {-0.446, 0.25},
        {-0.219, 0.33},
        {0, 0.42},
        {0.21, 0.5},
        {0.438, 0.58},
        {0.659, 0.66},
        {1, 0.75}
    }
    local normalizedElevTrim = normalize_value(rawTrim, data)
    if normalizedElevTrim ~= normalizedServo[trimServo] then
        normalizedServo[trimServo] = normalizedElevTrim
        hw_message_port_send(servo_driver, servosMsg, "FLOAT[8]", {table.unpack(normalizedServo)})
    end
end

xpl_dataref_subscribe("sim/cockpit2/controls/elevator_trim", "FLOAT", elevatorTrim_callback)
