-- Minimum working example: Logging analog readings from CubeOrange+ ADC port to a .csv file on the Cube's SD card.
local file_name = "data_log.csv" -- Log file name
local analog_in = analog:channel()
local pot_voltage = {} -- Table to hold potentiometer voltage readings
local LOOP_DELAY_MS = 50 -- Delay between readings in milliseconds = 20Hz
local logging_enabled = false

if not analog_in:set_pin(8) then
  gcs:send_text(0, "Invalid analog pin")
end -- Sets ADC pin 8 for potentiometer input

file = io.open(file_name, "a")
if not file then
  error("Could not make file")
end -- Opens file for data logging

file:write('Time [ms], Potentiometer Voltage [V], Pitch, Yaw, Roll, Pitch Rate, Servo 1, Servo 2, Servo 3, Servo 4, Servo 5, AUX 3, AUX 5\n')
file:flush() -- Writes CSV header

local function open_file()
  file = io.open(file_name, "a")
end

local function close_file()
  if file then
    file:close()
    file = nil
  end
end

function update()
  pot_voltage[1] = analog_in:voltage_latest() -- Gets latest voltage reading from potentiometer
  gcs:send_named_float("POT_VOLT", pot_voltage[1])
  -- ----- Attitude -----

  -- ===== RC SWITCH (CH10) =====
  local rc10 = rc:get_pwm(10)
  --gcs:send_text(0, tostring(rc8))

  if rc10 > 1600 and not logging_enabled then
    open_file()
    logging_enabled = true
    gcs:send_text(0, "Lua logging STARTED")

  elseif rc10 < 1400 and logging_enabled then
    close_file()
    logging_enabled = false
    gcs:send_text(0, "Lua logging STOPPED")
  end

  if logging_enabled and file then
    local roll  = math.deg(ahrs:get_roll() or 0)
    local pitch = math.deg(ahrs:get_pitch() or 0)
    local yaw   = math.deg(ahrs:get_yaw() or 0)

    -- ----- Pitch rate -----
    local pitch_rate = 0
    local gyro = ahrs:get_gyro()
    if gyro then
      pitch_rate = math.deg(gyro:y())
    end

    -- ----- Servo outputs -----
    local servo1 = rc:get_pwm(1)
    local servo2 = rc:get_pwm(2)
    local servo3 = rc:get_pwm(3)
    local servo4 = rc:get_pwm(4)
    local servo5 = rc:get_pwm(5)
    local servo_aux3 = rc:get_pwm(11)
    local servo_aux5 = rc:get_pwm(13)
   

    file:write(tostring(millis()) .. ", " .. table.concat(pot_voltage) ..", ".. tostring(pitch) ..", ".. tostring(yaw)
    ..", ".. tostring(roll)..", ".. tostring(pitch_rate)..", ".. tostring(servo1)..", ".. tostring(servo2)..", ".. tostring(servo3)
    ..", ".. tostring(servo4)..", ".. tostring(servo5)..","..tostring(servo_aux3)..","..tostring(servo_aux5)..",".. "\n") -- Writes data with timestamp, in CSV format
    file:flush() -- Updates the file
  end
  return update, LOOP_DELAY_MS
end

return update()
