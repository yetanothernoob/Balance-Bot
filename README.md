# Balance-Bot
Source file for arduino solution for the inverted pendulum problem on a single axis using a PID controller for generating PWM signals.
Hardware used:
Microcontroller – Arduino UNO R3
IMU – MPU 6050
Motor Controller – L298N
High Torque DC Motors and Wheels

Connection:
MPU – Arduino
VCC - 5V
GND - GND
SCL - A5
SDA - A4
INT - PWM 2

L298N – Arduino
ENA - PWM 3
IN1 - PWM 4
IN2 - PWM 8
IN3 - PWM 5
IN4 - PWM 7
ENB - PWM 6

For more details refer to the attached pdf.
