#ifndef EX_ESP_PINS_H
#define EX_ESP_PINS_H

// GPIO0    : v starting conditions - 10K to + 3.3V, or shorted to ground for flashing
// GPIO1    : x U0TXD (Serial1)
// GPIO2    : v starting conditions - 10K to + 3.3V
// GPIO3    : x U0RXD (Serial1)
// GPIO4    : v
// GPIO5    : v
// GPIO6    : x SD_CLK
// GPIO7    : x SD_DATA0
// GPIO8    : x SD_DATA1
// GPIO9    : x SD_DATA2
// GPIO10   : x SD_DATA3
// GPIO11   : x SD_CMD
// GPIO12   : v
// GPIO13   : v
// GPIO14   : v
// GPIO15   : v starting conditions - 10K on the ground
// GPIO16   : v
// T_OUT    : v Analog pin (17)

#define MOTORS_DIR_PIN       14
#define MOTOR1_STEP_PIN      12
#define MOTOR2_STEP_PIN      13
#define MOTORS_ENABLE_PIN    15  // 10K resistor to the ground can not be installed because the resistors are integrated into the stepper motor driver

#define SONAR_TRIG_PIN       16  // XPD !!! This and no other !!!
#define SCL_PIN              5   // or 4, for the ESP-07 (board divorced is not correct)
#define SDA_PIN              4   // or 5, for the ESP-07 (board divorced is not correct)
#define SONAR_ECHO_PIN       0

#define SERVO1_PIN           2   // 
#define SERVO2_PIN           3   // RXD
#define BATTERY_PIN          17  // ADC

#endif  // EX_ESP_PINS_H
