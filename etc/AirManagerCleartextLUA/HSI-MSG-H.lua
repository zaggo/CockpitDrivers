
local forceRefresh

local MessageId = {
    HSI                = 1,
    Home               = 2,
    SetInitialDegrees  = 3,
    SetDegreesCDI      = 4,
    SetDegreesCompass  = 5
}

local HSIPayload= {
    cdiDegree = 1,
    compassDegree = 2,
    hdgDegree = 3,
    vorOffset = 4,
    vsiOffset = 5,
    fromTo = 6
}

local busVoltageOk = false
local busVoltageInitialized = false
local hsiValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
local current_cdi_degrees = 0.0
local current_hdg_degrees = 0.0

function new_message(id, payload)
  print("Received Message id "..id)
  if id == MessageId.Home then
    print("HSI did home")
    local hdg = current_hdg_degrees
    if hdg<0 then
        hdg = hdg+360
    end
    hw_message_port_send(hsi_driver, MessageId.SetInitialDegrees, "FLOAT[2]", {current_cdi_degrees, hdg})
    forceRefresh()
  end
  if id == MessageId.SetDegreesCDI then
      print("Received DegreesCDI: "..payload)
      xpl_dataref_write("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot", "FLOAT", payload)
  end
  if id == MessageId.SetDegreesCompass then
      local hdg = payload
      if hdg > 180 then
         hdg = hdg-360
      end
      print("Received DegreesCompass: "..payload)
      xpl_dataref_write("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", "FLOAT", hdg)
  end
end

hsi_driver = hw_message_port_add("ARDUINO_NANO_H", new_message)
hw_message_port_send(hsi_driver, MessageId.Home, "INT", 1)

function compass_callback(deg)
    local dispDeg = -var_round(deg,2)
    if not busVoltageOk then
        dispDeg = 0
    end
    if hsiValues[HSIPayload.compassDegree] ~= dispDeg then
        hsiValues[HSIPayload.compassDegree] = dispDeg
        -- print("Compass: "..deg.." display: "..dispDeg)
        hw_message_port_send(hsi_driver, MessageId.HSI, "FLOAT[6]", {table.unpack(hsiValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot", "FLOAT", compass_callback)

function cdi_callback(deg)
    local dispDeg = (360-var_round(deg,2))%360
    current_cdi_degrees = deg
    if hsiValues[HSIPayload.cdiDegree] ~= dispDeg then
        hsiValues[HSIPayload.cdiDegree] = dispDeg
        print("cdi: "..deg.." display: "..dispDeg)
        hw_message_port_send(hsi_driver, MessageId.HSI, "FLOAT[6]", {table.unpack(hsiValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot", "FLOAT", cdi_callback)

function hdg_callback(deg)
    local dispDeg = (360-var_round(deg,2))%360
    current_hdg_degrees = dispDeg
    if hsiValues[HSIPayload.hdgDegree] ~= dispDeg then
        hsiValues[HSIPayload.hdgDegree] = dispDeg
        print("Heading: "..deg.." display: "..dispDeg)
        hw_message_port_send(hsi_driver, MessageId.HSI, "FLOAT[6]", {table.unpack(hsiValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", "FLOAT", hdg_callback)

function frto_callback(val)
    local dispVal = val*1.0
    if not busVoltageOk then
        dispVal = 0
    end
    if hsiValues[HSIPayload.fromTo] ~= dispVal then
        hsiValues[HSIPayload.fromTo] = dispVal
        --print("FromTo: "..val.." display: "..dispVal)
        hw_message_port_send(hsi_driver, MessageId.HSI, "FLOAT[6]", {table.unpack(hsiValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit/radios/nav1_fromto", "INT", frto_callback)

function vor_callback(val)
    local dispVal = val*10
    if not busVoltageOk then
        dispVal = 0
    end
    if hsiValues[HSIPayload.vorOffset] ~= dispVal then
        hsiValues[HSIPayload.vorOffset] = dispVal
        --print("vor: "..val.." display: "..dispVal)
        hw_message_port_send(hsi_driver, MessageId.HSI, "FLOAT[6]", {table.unpack(hsiValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit/radios/nav1_hdef_dot", "FLOAT", vor_callback)

function vsi_callback(val)
    local dispVal = val*10
    if not busVoltageOk then
        dispVal = 0
    end
    if hsiValues[HSIPayload.vsiOffset] ~= dispVal then
        hsiValues[HSIPayload.vsiOffset] = dispVal
        --print("vsi: "..val.." display: "..dispVal)
        hw_message_port_send(hsi_driver, MessageId.HSI, "FLOAT[6]", {table.unpack(hsiValues)})
    end
end
xpl_dataref_subscribe("sim/cockpit/radios/nav1_vdef_dot", "FLOAT", vsi_callback)

-- BUS VOLTAGE
function bus_volts_callback(volts)
    local busVolts = volts[1]
    local reloadNeeded = false
    if busVolts<6 then
        reloadNeeded = busVoltageOk or (not busVoltageInitialized)
        busVoltageOk = false
    else
        reloadNeeded = not busVoltageOk or (not busVoltageInitialized)
        busVoltageOk = true
    end
    
    if reloadNeeded then
        busVoltageInitialized = true
        print("busVoltage = "..busVolts.." ok = "..tostring(busVoltageOk))
        forceRefresh() 
    end
end
xpl_dataref_subscribe("sim/cockpit2/electrical/bus_volts", "FLOAT[6]", bus_volts_callback)

forceRefresh = function()
    print("ForceRefresh")
    request_callback(bus_volts_callback)
    request_callback(vsi_callback)
    request_callback(vor_callback)
    request_callback(frto_callback)
    request_callback(hdg_callback)
    request_callback(cdi_callback)
    request_callback(compass_callback)
end
