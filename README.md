# HidRTCwatch

feather M0 with nRF52 (Bluetooth LE hid commands), RGB oled SSD1331, MPU-6050 (Shake to wake up) and RTC DS3231M clock.

# This Branch (Pomodoro Technique)

- Speaker on pin 10 replaced by a switch
- Switch device to Pomodoro Mode

## Pomodoro Mode

- Timer always seen

### front, left, right button and Poti

- Poti: Timer adjust
- front button: Timer start
- right button: force Display refresh
- left: set/clear tomato (1 tomoato for each pomodori)
- Poti to max and right button: dim (power save) mode
- is Timer below 10 minutes: numbers switch from white to green
- is Timer below 5 minutes: numbers switch from green to orange
- Color of a pause: blue
- pause max 5 minutes, after 4 Pomodori up to 25 minutes

## Normal Mode

- press button or shake to wake up
- battery bar
- day, year, month ...
- 2 buttons for keyboard commands
- poti to select the 2-button keys
- mouse mode via gyroscope

### Selection

- BIG CLOCK always on! (2 buttons: dim display, normal display)
- Mouse left, right click (takes many power)
- cursor left, right
- cursor up, down
- page up, down
- DEL, space
- volume up, down
- media pref, next
- media stop, play (pause)
- off,on LED Stroboscope
- off,on LED (white, blue or UV)
- off,on LED blink Police (blue-UV)
- off,on LED Stroboscope
- stop,start and set countdown alarm (-30sec, +30sec) function
- set minutes (down,up)
- set hours (down,up)

## Hardware

https://www.exp-tech.de/plattformen/arm/atmel/6866/adafruit-feather-m0-bluefruit-le

