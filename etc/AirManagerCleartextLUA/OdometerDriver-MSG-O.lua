function new_message(id, payload)
  -- Do something with the message from the Arduino
end

display_driver = hw_message_port_add("ARDUINO_NANO_O", new_message)

-- MessageIDs
kTachTime = 6

tach1000 = 0
tach100 = 0
tach10 = 0
tach1 = 0
tackTenths = 0
tachHundredths = 0

function tack1000_callback(value)
    if value ~= tach1000 then
        tach1000 = value
        hw_message_port_send(display_driver, kTachTime, "FLOAT[6]", { tach1000, tach100, tach10, tach1, tachTenths, tachHundredths })
    end
end
xpl_dataref_subscribe("VFLYTEAIR/tach/TachTimeHrs1000", "INT", tack1000_callback)

function tack100_callback(value)
    if value ~= tach100 then
        tach100 = value
        hw_message_port_send(display_driver, kTachTime, "FLOAT[6]", { tach1000, tach100, tach10, tach1, tachTenths, tachHundredths })
    end
end
xpl_dataref_subscribe("VFLYTEAIR/tach/TachTimeHrs100", "INT", tack100_callback)

function tack10_callback(value)
    if value ~= tach10 then
        tach10 = value
        hw_message_port_send(display_driver, kTachTime, "FLOAT[6]", { tach1000, tach100, tach10, tach1, tachTenths, tachHundredths })
    end
end
xpl_dataref_subscribe("VFLYTEAIR/tach/TachTimeHrs10", "INT", tack10_callback)

function tack1_callback(value)
    if value ~= tach1 then
        tach1 = value
        hw_message_port_send(display_driver, kTachTime, "FLOAT[6]", { tach1000, tach100, tach10, tach1, tachTenths, tachHundredths })
    end
end
xpl_dataref_subscribe("VFLYTEAIR/tach/TachTimeHrs1", "INT", tack1_callback)

function tackTenths_callback(value)
    local int = math.floor(value)
    if int ~= tachTenths then
        tachTenths = int
        hw_message_port_send(display_driver, kTachTime, "FLOAT[6]", { tach1000, tach100, tach10, tach1, tachTenths, tachHundredths })
    end
end
xpl_dataref_subscribe("VFLYTEAIR/tach/TachTimeTenths", "FLOAT", tackTenths_callback)

function tackHundredths_callback(value)
    local rounded = var_round(value, 2)
    if value-math.floor(value) >= 0.9 then
        rounded = var_round(value, 3)
    end
    if rounded ~= tachHundredths then
        tachHundredths = rounded
        hw_message_port_send(display_driver, kTachTime, "FLOAT[6]", { tach1000, tach100, tach10, tach1, tachTenths, tachHundredths })
    end
end
xpl_dataref_subscribe("VFLYTEAIR/tach/TachTimeHundredths", "FLOAT", tackHundredths_callback)

request_callback(tack1000_callback)  
request_callback(tack100_callback)
request_callback(tack10_callback)
request_callback(tack1_callback)
request_callback(tackTenths_callback)
request_callback(tackHundredths_callback)
 
