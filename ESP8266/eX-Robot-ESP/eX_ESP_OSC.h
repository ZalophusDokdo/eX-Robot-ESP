#ifndef EX_ESP_OSC_H
#define EX_ESP_OSC_H

#include "eX_ESP_WiFi.h"

float    fader[8], slide[8];                          // [4] #########################
float    rotary[8];                         // add rotary ##################
float    xy1_x[8], xy1_y[8], xy2_x[8], xy2_y[8];
float    multixy1_x[8], multixy1_y[8];      // add multixy #################
//int      page = 1, push[8], toggle[8];      // [2] #########################
int      page = 1;
int      push[8];
int      toggle[8];
boolean  ChangePage, RequestBAT, AutoMode;

float    OSC_Extract_Param(int msgSize)
{
  u.Buff[0] = (unsigned char)PacketBuffer[msgSize-1];
  u.Buff[1] = (unsigned char)PacketBuffer[msgSize-2];
  u.Buff[2] = (unsigned char)PacketBuffer[msgSize-3];
  u.Buff[3] = (unsigned char)PacketBuffer[msgSize-4];
  return(u.d);
}

boolean  OSC_MSG_Read()
{
  if (!WiFi_MSG_Read()) return false;

  if (PacketBuffer[1]=='p')  // command PING
  {
    RequestBAT = true;
    return false;
  }
  if (PacketBuffer[0]=='/') page = int(PacketBuffer[1]) - 48;
  if (!(PacketBuffer[2]=='/')) 
  {
    ChangePage = true;
    return true;
  }
  if (PacketBuffer[3]=='f')  // command FADER
  {
    fader[int(PacketBuffer[8] - 49)] = OSC_Extract_Param(PacketSize);
    return true;
  }
  if (PacketBuffer[3]=='t')  // command TOGGLE
  {
    toggle[int(PacketBuffer[9] - 49)] = int(OSC_Extract_Param(PacketSize));
    return true;
  }
  if (PacketBuffer[3]=='p')  // command PUSH
  {
    push[int(PacketBuffer[7] - 49)] = int(OSC_Extract_Param(PacketSize));
    return true;
  } 
  if (PacketBuffer[3]=='r')  // command ROTARY #############################
  {
    rotary[int(PacketBuffer[9] - 49)] = OSC_Extract_Param(PacketSize);
    return true;
  } 
  if (PacketBuffer[12]=='x')  // command MultiXY1_x #########################
  {
    multixy1_x[int(PacketBuffer[13] - 49)] = OSC_Extract_Param(PacketSize);
    return true;
  } 
  if (PacketBuffer[12]=='y')  // command MultiXY1_y #########################
  {
    multixy1_y[int(PacketBuffer[13] - 49)] = OSC_Extract_Param(PacketSize);
    return true;
  } 
  return false;
}

#endif  // EX_ESP_OSC_H
