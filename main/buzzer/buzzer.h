#ifndef BUZZER_H
#define BUZZER_H

/*
If using a 2N2222 transistor:

Base: Connect to GPIO 23 (through a 1kΩ resistor).
Collector: Connect to one terminal of the buzzer.
Emitter: Connect to GND.
Other terminal of the buzzer: Connect to +5V.
Flyback Diode: Place across the buzzer terminals.

*/
void activate_buzzer(void);

#endif // BUZZER_H
