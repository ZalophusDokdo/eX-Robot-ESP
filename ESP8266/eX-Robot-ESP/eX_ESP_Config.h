#ifndef EX_ESP_CONFIG_H
#define EX_ESP_CONFIG_H

//#define  SOFTAP    // Soft AP mode #########################################
#define  STATION   // Station mode #########################################
//#define  USE_UART

#ifndef USE_UART
  #define USE_2_SERVOS
#endif

///  I2C bus parameters and UART     ///
#define  I2C_SPEED                  400000         // 400kHz I2C speed
#define  SERIAL_SPEED               115200
#define  K_bat                      0.0428         // 0.004581469370719 ####

///  The module parameters MPU6050   ///
//============= start ================//
#define  ACCEL_SCALE_G              8192           // (2G range) G = 8192
#define  ACCEL_WEIGHT               0.01
#define  GYRO_BIAS_WEIGHT           0.005
// MPU6000 sensibility (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define  Gyro_Gain                  0.03048
#define  Gyro_Scaled(x)             x * Gyro_Gain  // Return the scaled gyro raw data in degrees per second
#define  RAD2GRAD                   57.2957795
#define  GRAD2RAD                   0.01745329251994329576923690768489
//============== end =================//

///  Set the parameters of step motor and driver type  ///
//============= start ================//
#define  MICROSTEPPING              16             // 8 or 16, 32 for 1/8 or 1/16 driver microstepping (default:16)
#define  ANGLE_PER_STEP             1.8            // 0.9, 1.8
#define  K_MOTOR_SPEED              MICROSTEPPING * 360 / ANGLE_PER_STEP 
#define  PRE_DIR_STROBE             27             // 25 - 640 ns, 27 - 690ns, 28 - 740 ns
//============== end =================//

///  Constructive Section            ///
//============= start ================//
// In atorskoy version used wheel diameter D = 100mm
// So for one wheel revolution drives the robot away Pi * D = 314.15652mm
// One motor revolution - it is 200 steps to 1.8 degrees or 1600 microsteps at 1/8
// Control the maximum speed - 500
// This value is multiplied by 23 give the motor speed 
// And defined period following the control pulses 173,913 interval 0.5 microseconds
// Or a period of 86.9565ms
// That sootvetstvavalo frequency 11.5kHz (it is 500 * 23 = 11500)
// At these frequencies, the motor made 7.1875 complete revolutions per second
// Which corresponded to the linear velocity of 2258mm/sec (135.48m/min) (8.1288km/h)
//
#define  MAX_SPEED_LINEAR           8.5            // km/h
#define  VHEEL_DIAMETR              70             // 108mm ##################

#define  MAX_TURN                   MAX_SPEED_LINEAR * 1000000 / 3600 / VHEEL_DIAMETR / PI  // revolutions per second
#define  MAX_FREQUENCY              MAX_TURN * 360 / ANGLE_PER_STEP * MICROSTEPPING         // maximum frequency STEP(Hz) pulse
//============== end =================//

///  Handling options                ///
//============= start ================//
#define  ZERO_SPEED                 5000          // 25000
#define  MAX_ACCEL                  8              // 8 // Maximun motor acceleration (MAX RECOMMENDED VALUE: 8) (default:7)
#define  MAX_CONTROL_OUTPUT         500

// NORMAL MODE = smooth, moderately
#define  MAX_THROTTLE               480            // 480 throttle accellerator speed
#define  MAX_STEERING               150            // 130 max steer, understeer
#define  MAX_TARGET_ANGLE           10             // 12 ### // max tilt angle

// PRO MODE = more aggressive
#define  MAX_THROTTLE_PRO           780            // 680
#define  MAX_STEERING_PRO           300            // 250
#define  MAX_TARGET_ANGLE_PRO       20             // 20
//============== end =================//

///  PID-parameters                  ///
//============= start ================//
// The default management conditions 
#define  KP                         0.19           // 0.19 // alternative values: 0.20, 0.22
#define  KD                         28             // 30   // 26 28
#define  KP_THROTTLE                0.08           // 0.07 // 0.065, 0.08
#define  KI_THROTTLE                0.05           // 0.04 // 0.05
// Gain control when the robot picked up from a lying position
// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define  KP_RAISEUP                 0.16
#define  KD_RAISEUP                 40             // 40
#define  KP_THROTTLE_RAISEUP        0              // is not controlled when lifting speed motors
#define  KI_THROTTLE_RAISEUP        0.0
#define  ITERM_MAX_ERROR            40             // 40   // Iterm windup constants
#define  ITERM_MAX                  8000           // 5000
//============== end =================//

///  Servo drive parameters          ///
// ============= Start ===============//
// Maximum period of 20 milliseconds and cycle duration mkc 10 = 2000 cycles
// Positive gate control pulse ranges:
// Minimum - 0.6ms, average - 1.5ms and the largest - 2.4ms or periods 60-150-240
//  
#define  SERVO_AUX_NEUTRO           150            // neutral arm servo drive
#define  SERVO_MIN_PULSEWIDTH       80
#define  SERVO_MAX_PULSEWIDTH       220

#define  SERVO_NEUTRAL              150            // Servo neutral positions ####
// Normal servo range (servo goes from SERVO_NETRAL - (SERVO_RANGE / 2) to SERVO_NEUTRAL + (SERVO_RANGE / 2)
#define  MAX_SERVO_RANGE            120            // 90
// Range when PRO mode is pressed (usually more servo range)
#define  MAX_SERVO_RANGE_PRO        180            // 100
//============== end =================//
#endif  // EX_ESP_CONFIG_H
