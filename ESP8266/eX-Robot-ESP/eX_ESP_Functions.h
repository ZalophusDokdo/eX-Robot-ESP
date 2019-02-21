#ifndef EX_ESP_FUNCTIONS_H
#define EX_ESP_FUNCTIONS_H

float PID_errorSum;
float PID_errorOld  = 0;
float PID_errorOld2 = 0;
float setPointOld   = 0;

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error         = setPoint - input;

  // Kd is implemented in two parts
  // The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  // And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp*error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT;  // + error - PID_error_Old2
  //Serial.print(Kd * (error - PID_errorOld));
  //Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld  = input;         // error for Kd is only the input component
  setPointOld   = setPoint;
  return(output);
}

// PI controller implementation (Proportional, integral). DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error         = setPoint - input;
  PID_errorSum  += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum  = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);
  
  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT * 0.001;  // DT is in miliseconds...
  return(output);
}

#endif  // EX_ESP_FUNCTIONS_H
