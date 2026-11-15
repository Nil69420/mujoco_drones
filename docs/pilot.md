# Pilot input: flying with a gamepad or a radio

`pilot_tool` reads a game controller or an EdgeTX/OpenTX radio in USB joystick mode through Linux evdev (no extra libraries), identifies it, maps its sticks to roll, pitch, yaw and throttle, and can drive Betaflight SITL over its UDP RC input.

```sh
pilot_tool profiles                      # built-in controller profiles
pilot_tool list                          # controllers found on this machine
pilot_tool monitor /dev/input/event20    # live sticks, arm and mode state
pilot_tool calibrate /dev/input/event20 my.profile
pilot_tool betaflight /dev/input/event20 [my.profile] [udp-port]
```

## Identification

`pilot_tool list` matches each controller by USB vendor and product ID, and EdgeTX, OpenTX and RadioMaster radios by name. Built in: Sony DualSense, DualSense Edge, DualShock 4, Xbox One and Series (USB and Bluetooth), Logitech F310 and F710, and generic EdgeTX/OpenTX/RadioMaster radios. **Every built-in profile is marked unverified**: they come from the kernel drivers' documented axis conventions, not from a device on the project's bench. Check the directions with `monitor` before flying. The radio profiles match by name because the USB IDs of a radio in joystick mode depend on the firmware; a RadioMaster GX12 has not been confirmed.

An unknown controller gets the generic gamepad profile. `pilot_tool calibrate` fixes that: it asks you to push each stick in turn, finds the axis that moved and whether it is inverted, asks whether the throttle stick stays where it is left (a radio) and which buttons arm and toggle the flight mode, and writes a profile file. A profile is a plain text file (`key = value`, see `pilot_profile_format` for every key); to add your own controller by hand, copy the output of `calibrate` and edit it.

## Behaviour

- Sticks: deadzone, then an expo curve (`expo` 0 is linear). Mode 2 layout: left stick yaw and throttle, right stick roll and pitch.
- Throttle: `integrate` (gamepads: the stick sets the rate and the throttle holds where you leave it), `direct` (radios) or `trigger`.
- Arming: gamepad button held for half a second with the throttle near zero; pressing it again disarms immediately. Radio: an arm switch (an axis) arms only if the throttle is near zero at that moment and disarms as soon as the switch goes off.
- Failsafe: if the controller disconnects, sticks go to neutral, the throttle to zero and the vehicle disarms; reconnecting never re-arms automatically, and the integrated throttle restarts at zero.

## Output

`pilot_tool betaflight` sends the 16-channel RC packet Betaflight SITL listens for on UDP 9004 (roll, pitch, throttle, yaw, AUX1 arm, AUX2 mode, then the aux axes), at 100 Hz. Other outputs (MAVLink manual control, the simulator's own setpoints, a Renoir topic) are planned in the roadmap (workstream C7).

## Access to the device

The evdev nodes belong to the `input` group. At the console the login seat normally grants access to controllers; over SSH, or if `pilot_tool list` reports unreadable devices, add your user to the `input` group.

## Not implemented

SDL as an alternative backend, rumble, lightbar and motion-sensor control for the DualSense, hot-plug discovery while running, and the other outputs above. The tests cover profile parsing and formatting (and a fuzz target), stick shaping, throttle modes, arming and failsafe rules, calibration, and event reading through a pipe; nothing has been run against a physical controller.
