
NIDEC 24V brushless motor
NIDEC pinout, from left to right (when the connector is directed towards you and the rotor is pointing up):
- Power positive
- Power negative
- Start/stop
- PWM
- forward/reverse
- encoder power supply (5V)
- encoder A
- encoder B

Start/Stop: High mean on, low means stop.
Encoders: There's 25 holes for each encoder, giving us a precision of 360/(2*2*25) when using both encoders.
Direction: Low means Counter clockwise rotation when axis is pointing towards you.
PWM: always low means highest speed.


Släpring connections, from red to purple
Power positive
Ground
PWM
5V
Encoder A
Forward Reverse

Motor   Function    Släpring
Brown   12V         Orange
Red     Ground      Black
Orange  Start/Stop  Green
Yellow  PWM         Gray
Green   Forward/R   Yellow
Blue    5V          Purple
Purple  Encoder A   White
