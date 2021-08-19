# TL;DR

cat awesome.gcode | netcat ender 1212

Printer turns on, print starts, after print is done printer truns off.

Enjoy!

# Setup

Connect ESP8266 to printer TX<->RX, RX<->TX and GND<->GND

# Example
```yaml


external_components:
  source: github://ABrocksdorff/wifiprint

logger :
    baud_rate: 0
    level: INFO

 switch:
  - platform: gpio
    name: "Relay"
    restore_mode: ALWAYS_OFF
    id: relay
    pin: GPIO12 #D6
    inverted: True

 uart:
  tx_pin: GPIO01
  rx_pin: GPIO03
  baud_rate: 115200

 gcodestreamer:
    name: "Ender"
    port: 1212
    filters:
      - delayed_off: 600s
    on_press:
      - switch.turn_on: relay
    on_release:
      - switch.turn_off: relay

```

# Misc

longest print so far 2 days 8 hours

up to 100 lines/s gcode on ESP8266

# known problems

if your wifi dies so does your print :-(

print cannot be aborted at the printer

