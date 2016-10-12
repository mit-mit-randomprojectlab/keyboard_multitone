# keyboard_multitone
sketch to test basic keyboard matrix using teensy and using a custom implementation of "Tone" (square-wave generation) that uses all four PIT timers to generate up to four simultaneous tones.

When connecting up the Teensy (I'm using a Teensy 3.2), connect pins 8, 17, 18 and 19 each to a 330 ohm resistor, then to a small speaker (I'm using a 4 ohm, 0.5W). You can configure the row and column pins for the switch matrix implementation to however your key board is setup: see lines 294, 295: my keyboard has 49 keys, in rows of 6, except for the last key which is on it's own. 

Be wary of how the rows/columns are setup: test all diode orientations etc. before running, otherwise risk shorting the Teensy.
