/*
  GSM_G510.h - Library for GSM_G510
  Created by Wojciech Tarnawski, 4.2015.
  Released into the public domain.
*/
#ifndef GSM_G510_h
#define GSM_G510_h

#include "Arduino.h"
#include "SoftwareSerial.h"

class GSM_G510
{
  public:
    GSM_G510(int rx, int tx, int on);
	void init();
	
	boolean isConnecting();
	boolean sendSms(char* phoneNumber, char* message);
	float getSignalStrength();
    float getIdent();
  private:
    SoftwareSerial serial; // RX, TX
	int on;
};

#endif

