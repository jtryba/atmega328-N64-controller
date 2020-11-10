# atmega328-N64-controller
A stand alone replacement to the Nintendo CNT-NUS N64 controller chip, with built in rumble, for use in portables.
Tested on 328, 328P, and 168 (all @ 16 Mhz)

*If memory pack support isnt required just use "N64_controller_replacement.ino"
*Memory pack emulation is WIP and currently does not work.
*Rumble is not working with goldeneye

Based off: Gamecube controller to Nintendo 64 adapter
by Andrew Brown
https://github.com/brownan/Gamecube-N64-Controller/blob/master/gamecube.ino
Thank you Andrew


Rewritten so the atmega328 can be used as a stand alone replacement to the Nintendo CNT-NUS N64 controller chip, with built in rumble, for use in portables.

To use, hook up the following to the Arduino Atmega328
Digital I/O 8:  N64 serial line
Digital I/O 10: 220 ohm resistor to 2n2222 base, emitter to ground, collector to motor using 1N4007 flyback diode accross motor legs, and finally motor to 5v
All appropriate grounding and power lines, joystick, and all buttons (all active low, see pin definitions in .ino)

Shoutout to <a href=https://bitbuilt.net/forums/index.php?members/pears.6450/>pears</a> from <a href=https://bitbuilt.net>bitbuilt.net</a> for testing.
