// eX-Robot Balancing Robot with stepper drive
// Author: Oleg Kravtchenko --> Modified by Zalophus Dokdo
// Date: 26/10/2015         --> 22/02/2017
// License: GPL v2

// To create a balancing robot is used: 
// - 1 unit WiFi ESP8266
// - 1 DMP processor MPU6050
// - 2 Drivers Stepper Motors A4988
// - 2 Stepper Motors NEMA17 17HS2408 (0.6A, 1.8 deg / step)
// Optional:
// - 1 Micro 9g Servo-Drive
// - 1 Distance sensor (sonar) HC-SR04
// - 1 MP3 sound module
// - N RGB LED

#include <ESP8266WiFi.h>
#include "eX_ESP_Config.h"
#include "eX_ESP_Pins.h"
#include "eX_ESP_WiFi.h"
#include "eX_ESP_OSC.h"
#include "eX_ESP_MPU6050.h"
#include "eX_ESP_Functions.h"
#include "eX_ESP_Timing_Engine.h"

//#include <Arduino.h>
bool       robot_shutdown   = false;   // Robot shutdown flag
bool       robot_pro_mode;             // Robot_mode = false - Normal mode, True - Pro mode
bool       newControlParameters        = false;  //#########################
bool       modifing_control_parameters = false;  //#########################

long       timer_old_value;
long       timer_value;
float      dt;
    
float      bat_level;
float      bat_level_bar;  //###############################################
float      dist_value;

float      throttle;
float      steering;

float      max_throttle     = MAX_THROTTLE;
float      max_steering     = MAX_STEERING;
float      max_target_angle = MAX_TARGET_ANGLE;
float      Kp               = KP;
float      Kd               = KD;
float      Kp_thr           = KP_THROTTLE;
float      Ki_thr           = KI_THROTTLE;
float      Kp_user          = KP;
float      Kd_user          = KD;
float      Kp_thr_user      = KP_THROTTLE;
float      Ki_thr_user      = KI_THROTTLE;

float      angle_adjusted;             // Angle of the robot (used for stability control)
float      angle_adjusted_Old;
int16_t    actual_robot_speed;         // overall robot speed (measured from steppers speed)
int16_t    actual_robot_speed_Old;    
float      estimated_speed_filtered;   // Estimated robot speed
float      target_angle;
float      control_output;

int16_t    motor1;
int16_t    motor2;

uint16_t   loop_counter = 0;

int servo1;                      //#########################################
int servo2;
int max_servo1;
int max_servo2;
int servo_mode;                  //#########################################

// Read control PID parameters from user. This is only for advanced users that want to "play" with the controllers...
void readControlParameters()
{
  // Parameter initialization (first time we enter page2)
  if ((page == 2) && (!modifing_control_parameters))
  {
    fader[0]  = 0.5;
    fader[1]  = 0.5;
    fader[2]  = 0.5;
    fader[3]  = 0.0;
    //toggle[0] = 0;
    modifing_control_parameters = true;
  }
  // Parameters Mode (page2 controls)
  // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fader1,2,3,4)
  // Now we need to adjust all the parameters all the times because we don´t know what parameter has been moved
  if (page == 2)
  {
    Kp_user     = KP * 2 * fader[0];
    Kd_user     = KD * 2 * fader[1];
    Kp_thr_user = KP_THROTTLE * 2 * fader[2];
    Ki_thr_user = (KI_THROTTLE + 0.1) * 2 * fader[3];

    // Kill robot => Sleep
    while (toggle[1])
    {
      //Reset external parameters
      mpuResetFIFO();
      PID_errorSum = 0;
      timer_old_value = millis();
      te_SetMotorsSpeed(0,0);
      OSC_MSG_Read();
    }
    newControlParameters = true;
  }
  if ((newControlParameters) && (!modifing_control_parameters))
  {
    // Reset parameters
    fader[0] = 0.5;
    fader[1] = 0.5;
    //toggle[0] = 0;
    newControlParameters = false;
  }
}

void setup() 
{
#ifdef USE_UART
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println("\n\n!!! eX-Robot-ESP ready !!!\n");
#endif
  WiFi_Start();
#ifdef USE_UART
  Serial.println("Wifi ready");
#endif
  te_Start();
  i2c_begin(SDA_PIN, SCL_PIN, I2C_SPEED);
  robot_pro_mode   = false;
  robot_shutdown   = false;
  if (mpu_Initialization() == 0)
    robot_shutdown = true;  
  timer_old_value  = millis();
  fader[0] = 0.5;
  fader[1] = 0.5;
  fader[2] = 0.5;
  fader[3] = 0.5;
  fader[4] = 0.5;
  fader[5] = 0.5;
  fader[6] = 0.5;
  fader[7] = 0.5;

  // Init servos ###########################################################
  Serial.println("Servo initialization...");
  delay(50);
  
  te_SetServo(SERVO_NEUTRAL, 0);
  te_SetServo(SERVO_NEUTRAL, 1);
  servo1     = SERVO_NEUTRAL;
  servo2     = SERVO_NEUTRAL;
  max_servo1 = MAX_SERVO_RANGE;
  max_servo2 = MAX_SERVO_RANGE;

  // Little motor vibration and servo move to indicate that robot is ready
  for (uint8_t k = 0; k < 3; k++)
  {
    te_SetServo(SERVO_NEUTRAL + 40, 0);
    te_SetServo(SERVO_NEUTRAL + 40, 1);
    delay(100);
    te_SetServo(SERVO_NEUTRAL - 40, 0);
    te_SetServo(SERVO_NEUTRAL - 40, 1);
    delay(100);
  }  
  te_SetServo(SERVO_NEUTRAL, 0);
  te_SetServo(SERVO_NEUTRAL, 1);
}

void loop()
{
  ArduinoOTA.handle();  // OTA ###########################################

  if (robot_shutdown)
    return;
  if (OSC_MSG_Read())
  {
    if (page == 1)    // Get commands from user (PAGE1 are user commands: throttle, steering...)
    {
      throttle = (fader[0] - 0.5) * max_throttle;
      // We add some exponential on steering to smooth the center band
      steering = fader[1] - 0.5;
      if (steering > 0)
      {
        steering = ( steering * steering + 0.5 * steering) * max_steering;
      }
      else
      {
        steering = (-steering * steering + 0.5 * steering) * max_steering;
      }

      // Auto reset ########################################################
      if (!fader[0])
      {
        fader[0] = 0.5;
        WiFi_MSG_Send_Float("/1/fader1\0\0\0,f\0\0\0\0\0\0",20,fader[0]);
        throttle = 0;
      }
      if (!fader[1])
      {
        fader[1] = 0.5;
        WiFi_MSG_Send_Float("/1/fader2\0\0\0,f\0\0\0\0\0\0",20,fader[1]);
        steering = 0;
      }
      // Auto reset ########################################################

      modifing_control_parameters = false;  //##############################
      if ((!robot_pro_mode) && (toggle[0] == 1))
      {
        // Change to PRO mode
        max_throttle     = MAX_THROTTLE_PRO;
        max_steering     = MAX_STEERING_PRO;
        max_target_angle = MAX_TARGET_ANGLE_PRO;
        robot_pro_mode   = true;    
      }
      if ((robot_pro_mode) && (toggle[0] == 0))
      {
        // Change to NORMAL mode
        max_throttle     = MAX_THROTTLE;
        max_steering     = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        robot_pro_mode   = false;
      }
      // Push2 reset controls to neutral position
      if (push[1])
      {
        fader[0] = 0.5;
        WiFi_MSG_Send_Float("/1/fader1\0\0\0,f\0\0\0\0\0\0",20,fader[0]);
        fader[1] = 0.5;
        WiFi_MSG_Send_Float("/1/fader2\0\0\0,f\0\0\0\0\0\0",20,fader[1]);
        // Reset external parameters #######################################
        mpuResetFIFO();
        PID_errorSum    = 0;
        control_output  = 0;
        timer_old_value = millis(); 
        te_SetMotorsSpeed(0,0);
        OSC_MSG_Read();
      }
      // Push3 is emergency stop ###########################################
      while (push[2])
      {
        PIN_HIGH(MOTORS_ENABLE_PIN);     // Driver is not active
        te_SetMotorsSpeed(0, 0);
        PID_errorSum   = 0;              // Reset PID I term
        control_output = 0;
      }
      // Push4 is Move servo arm 2 #########################################
#ifdef USE_2_SERVOS
      if (push[3])                          // Move arm 2
      {
        te_SetServo(SERVO_MIN_PULSEWIDTH + 10, 1);
      }
      else
      {
        te_SetServo(SERVO_AUX_NEUTRO, 1);
      }
#endif
      /*
      // RiseUp ############################################################
      if (push[4])                          // Push5
      {
        PIN_LOW(MOTORS_ENABLE_PIN);         // Active driver
        te_SetMotorsSpeed(motor1, motor2);
        PID_errorSum   = 0;                 // Reset PID I term
        control_output = 0;
        Kp     = KP_RAISEUP;                // CONTROL GAINS FOR RAISE UP
        Kd     = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP;
        Ki_thr = KI_THROTTLE_RAISEUP;
      }
      */
    }
    if (page == 2)      // set Page = 2 ###############################
    {
      throttle = 0;
      steering = 0;
      if (ChangePage)
      {
        ChangePage = false;
        fader[0] = Kp_user / KP / 2;
        WiFi_MSG_Send_Float("/2/fader1\0\0\0,f\0\0\0\0\0\0",20,fader[0]);
        fader[1] = Kd_user / KD / 2;
        WiFi_MSG_Send_Float("/2/fader2\0\0\0,f\0\0\0\0\0\0",20,fader[1]);
        fader[2] = Kp_thr_user / KP_THROTTLE / 2;
        WiFi_MSG_Send_Float("/2/fader3\0\0\0,f\0\0\0\0\0\0",20,fader[2]);
        fader[3] = Ki_thr_user / (KI_THROTTLE + 0.1) / 2;
        WiFi_MSG_Send_Float("/2/fader4\0\0\0,f\0\0\0\0\0\0",20,fader[3]);
      }
      else
      {
        Kp_user     = KP * 2 * fader[0];
        Kd_user     = KD * 2 * fader[1];
        Kp_thr_user = KP_THROTTLE * 2 * fader[2];
        Ki_thr_user = (KI_THROTTLE + 0.1) * 2 * fader[3];
      }
      while (toggle[1])               // toggle2 is Kill
      {
        // Reset external parameters
        mpuResetFIFO();
        PID_errorSum    = 0;
        control_output  = 0;
        timer_old_value = millis(); 
        te_SetMotorsSpeed(0,0);
        OSC_MSG_Read();
      }
      // Push1 is reset default setup ####################################
      if (push[0])
      {
        fader[0] = 0.5;
        fader[1] = 0.5;
        fader[2] = 0.5;
        fader[3] = 0.5;
        WiFi_MSG_Send_Float("/2/fader1\0\0\0,f\0\0\0\0\0\0",20,fader[0]);
        WiFi_MSG_Send_Float("/2/fader2\0\0\0,f\0\0\0\0\0\0",20,fader[1]);
        WiFi_MSG_Send_Float("/2/fader3\0\0\0,f\0\0\0\0\0\0",20,fader[2]);
        WiFi_MSG_Send_Float("/2/fader4\0\0\0,f\0\0\0\0\0\0",20,fader[3]);
        Kp_user     = KP * 2 * fader[0];
        Kd_user     = KD * 2 * fader[1];
        Kp_thr_user = KP_THROTTLE * 2 * fader[2];
        Ki_thr_user = (KI_THROTTLE + 0.1) * 2 * fader[3];
        OSC_MSG_Read();
      }
      // Push3 Reset #######################################################
      while (push[2])
      {
        PIN_HIGH(MOTORS_ENABLE_PIN);     // Driver is not active
        te_SetMotorsSpeed(0, 0);
        PID_errorSum   = 0;              // Reset PID I term
        control_output = 0;
      }
    }
    if ((page == 3) || (page == 4))     // set Page = 3 ###############################
    {
      throttle = 0;
      steering = 0;
      
      //servo1 = (multyxy1_x[0] - 0.5) * max_servo1 + SERVO_NEUTRAL;
      //servo2 = (-xy1_y[0] + 0.5) * max_servo2 + SERVO_NEUTRAL;
    
      servo1 = (fader[0] - 0.5) * max_servo1 + SERVO_NEUTRAL;
      servo2 = (fader[1] - 0.5) * max_servo2 + SERVO_NEUTRAL;
      if ((servo_mode == 1) || (toggle[0] == 1))
      {
        // Change to PRO mode
        max_servo1 = MAX_SERVO_RANGE_PRO;
        max_servo2 = MAX_SERVO_RANGE_PRO;
        //servo_mode = 1;
      }
      else if ((servo_mode == 0) || (toggle[0] == 0))
      {
        // Change to NORMAL mode
        max_servo1 = MAX_SERVO_RANGE;
        max_servo2 = MAX_SERVO_RANGE;
        //servo_mode = 0;
      }
      if (!fader[0])   // Auto center position #############################
      {
        fader[0] = 0.5;
        WiFi_MSG_Send_Float("/3/fader1\0\0\0,f\0\0\0\0\0\0",20,fader[0]);
        WiFi_MSG_Send_Float("/4/fader1\0\0\0,f\0\0\0\0\0\0",20,fader[0]);
        servo1 = SERVO_NEUTRAL;
      }
      if (!fader[1])
      {
        fader[1] = 0.5;
        WiFi_MSG_Send_Float("/3/fader2\0\0\0,f\0\0\0\0\0\0",20,fader[1]);
        WiFi_MSG_Send_Float("/4/fader2\0\0\0,f\0\0\0\0\0\0",20,fader[1]);
        servo2 = SERVO_NEUTRAL;
      }
      if (push[1])  // Sat No!! ############################################
      {
        for (uint8_t k = 0; k < 3; k++)
        {
          te_SetServo(SERVO_NEUTRAL + 40, 0);
          te_SetServo(SERVO_NEUTRAL + 40, 1);
          delay(1000);
          te_SetServo(SERVO_NEUTRAL - 40, 0);
          te_SetServo(SERVO_NEUTRAL - 40, 1);
          delay(1000);
        }
      }  
      if (push[2])  // Baby!! ############################################
      {
        for (uint8_t k = 0; k < 3; k++)
        {
          te_SetServo(SERVO_NEUTRAL + 60, 0);
          delay(1000);
          te_SetServo(SERVO_NEUTRAL - 60, 0);
          delay(1000);
          te_SetServo(SERVO_NEUTRAL + 60, 1);
          delay(1000);
          te_SetServo(SERVO_NEUTRAL - 60, 1);
          delay(1000);
        }
      }  
      te_SetServo(servo1, 0);
      te_SetServo(servo2, 1);
#ifdef USE_UART  //#########################################################
      Serial.print(servo1);
      Serial.print(" ");
      Serial.println(servo2);
#endif
    }
#ifdef USE_UART  //#########################################################
    Serial.print(OSC_MSG_Read());
#endif
  }
  // New DMP Orientation solution?
  fifoCount = mpuGetFIFOCount();
  if (fifoCount >= 18)
  {
    if (fifoCount > 18) // If we have more than one packet we take the easy path: discard the buffer and wait for the next one
    {
      mpuResetFIFO();
      return;
    }
    loop_counter++;
    timer_value        = millis();
    dt                 = (timer_value - timer_old_value);
    timer_old_value    = timer_value;
    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    angle_adjusted     = dmpGetPhi();

    if ((angle_adjusted < 54) && (angle_adjusted > -54))  // 74 Is Robot in working position?
    {
      // NORMAL MODE
      PIN_LOW(MOTORS_ENABLE_PIN);         // Active driver
      te_SetMotorsSpeed(motor1, motor2);  //################################
      
      // Push1 Move servo arm
      if (page == 1)                      // Move arm
      {
        if (push[0])                        // Move arm
        {
          te_SetServo(SERVO_MIN_PULSEWIDTH + 10, 0);
        }
        else
        {
          te_SetServo(SERVO_AUX_NEUTRO, 0);
        }
      }

      if ((angle_adjusted < 45) && (angle_adjusted > -45))  // 40 ############
      {
        Kp     = Kp_user;              // Get the control of the user's default
        Kd     = Kd_user; 
        Kp_thr = Kp_thr_user;
        Ki_thr = Ki_thr_user;
      }     
      else    // During the lifting of the robot in the working position, we use specific control parameters
      {
        Kp     = KP_RAISEUP;           // CONTROL GAINS FOR RAISE UP
        Kd     = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP; 
        Ki_thr = KI_THROTTLE_RAISEUP;
      } 
    }
    else                // Robot not ready (flat), angle > 70º => ROBOT OFF
    {
      PIN_HIGH(MOTORS_ENABLE_PIN);     // Driver is not active
      te_SetMotorsSpeed(0, 0);         //###################################
      PID_errorSum   = 0;              // Reset PID I term
      control_output = 0;
      Kp     = KP_RAISEUP;             // CONTROL GAINS FOR RAISE UP
      Kd     = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;

      // if we pulse push1 button we raise up the robot with the servo arm
      if (page == 1)
      {
        if (push[0])
        {
          // Because we know the robot orientation (face down of face up), we move the servo in the appropiate direction for raise up
          if (angle_adjusted > 0)
          {
            te_SetServo(SERVO_MIN_PULSEWIDTH, 0);
          }
          else
          {
            te_SetServo(SERVO_MAX_PULSEWIDTH, 0);
          }
        }
        else
        {
          te_SetServo(SERVO_AUX_NEUTRO, 0);
        }
      }
      // Check for new user control parameters #############################
      //readControlParameters();  //########################################
    }

    // We calculate the estimated speed of the robot
    // Design speed = angular velocity of the stepper motor (combined) - the angular velocity of the robot (the angle measured IMU)
    actual_robot_speed_Old   = actual_robot_speed;
    actual_robot_speed       = (speed_M1 + speed_M2) / 2;  // Positive: forward
    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0;  // 90 empirical coefficient obtained by adjusting the real indicators
    int16_t estimated_speed  = -actual_robot_speed_Old - angular_velocity;     //### // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;

    // SPEED CONTROL: This is a PI controller. 
    // input: user throttle
    // variable: estimated robot speed
    // output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr); 
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle);  // limited output

    // Stability control: This is a PD controller. 
    // input: robot target angle(from SPEED CONTROL)
    // variable: robot angle
    // output: Motor speed
    // We integrate reaction (summing) so that the output of the engine acceleration have, instead of its rotation speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    //control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);  // Limit max output from control ###

    // Enter the custom adjustment for the steering operation to the control signal
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);   
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);  
    te_SetMotorsSpeed(motor1, motor2);  
  }

  if (loop_counter > 400)
  {
    dist_value          = echo_value * 0.0125 / 58;       // in centimeters
    loop_counter        = 0;
    Serial.println(control_output);
  }

  if (RequestBAT)                                         // Ping ##########
  {
    bat_level           = analogRead(BATTERY_PIN) * K_bat;
    float bat_level_up  = roundf(bat_level * 100) / 100;  // Rounding ######
    bat_level_bar       = bat_level / 12;                 // 12v ###########
    RequestBAT          = false;
    if (page == 1)
    {
      WiFi_MSG_Send_Float("/1/label1\0\0\0,f\0\0\0\0\0\0",20, bat_level_up);      //### add roundf
      WiFi_MSG_Send_Float("/1/slide9\0\0\0,f\0\0\0\0\0\0",20, bat_level_bar);     //### add
      WiFi_MSG_Send_Float("/1/rotary1\0\0\0,f\0\0\0\0\0\0",20, bat_level_bar);    //### add
      WiFi_MSG_Send_Float("/1/label2\0\0\0,f\0\0\0\0\0\0",20, (int) dist_value);  //### add (int)
    }
    if (page == 2)
    {
      WiFi_MSG_Send_Float("/2/label1\0\0\0,f\0\0\0\0\0\0",20, bat_level_up);      //### add roundf
      WiFi_MSG_Send_Float("/2/slide9\0\0\0,f\0\0\0\0\0\0",20, bat_level_bar);     //### add
      WiFi_MSG_Send_Float("/2/label2\0\0\0,f\0\0\0\0\0\0",20, (int) dist_value);  //### add (int)
    }
    if (page == 3)
    {
      WiFi_MSG_Send_Float("/3/label1\0\0\0,f\0\0\0\0\0\0",20, bat_level_up);      //### add roundf
      WiFi_MSG_Send_Float("/3/label8\0\0\0,f\0\0\0\0\0\0",20, bat_level_up);      //### add roundf
      WiFi_MSG_Send_Float("/3/slide9\0\0\0,f\0\0\0\0\0\0",20, bat_level_bar);     //### add
      WiFi_MSG_Send_Float("/3/rotary8\0\0\0,f\0\0\0\0\0\0",20, bat_level_bar);    //### add
      WiFi_MSG_Send_Float("/3/label2\0\0\0,f\0\0\0\0\0\0",20, (int) dist_value);  //### add (int)
    }
    if (page == 4)
    {
      WiFi_MSG_Send_Float("/4/label1\0\0\0,f\0\0\0\0\0\0",20, bat_level_up);      //### add roundf
      WiFi_MSG_Send_Float("/4/label2\0\0\0,f\0\0\0\0\0\0",20, (int) dist_value);  //### add (int)
    }
  }
}

