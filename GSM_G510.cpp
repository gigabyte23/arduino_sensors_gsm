#include "GSM_G510.h"

GSM_G510::GSM_G510(int rx, int tx, int on) :
		serial(rx, tx), on(on) {

}

void GSM_G510::init() {

	serial.begin(9600);
	pinMode(on, OUTPUT);

	Serial.println("Laczenie z siecia GSM");
}

/*
 * false - brak polaczenia
 * true - polaczony
 */
boolean GSM_G510::isConnecting() {

	if (getSignalStrength() > 0) {

		return true;
	} else {
		return false;
	}

}

/*
 *	true	- sms wyslany
 *	false 	- sms nie wyslany
 */
boolean GSM_G510::sendSms(char* phoneNumber, char* message) {

	serial.setTimeout(3000);

	serial.flush();
	serial.write("\r\n");
	delay(10);

	serial.write("AT+CMGF=1\r\n");

	if (serial.findUntil("OK", "ERROR\r\n")) {
		serial.flush();

		serial.write("AT+CMGS=\"");
		serial.write(phoneNumber);
		serial.write("\"\r\n");
		serial.flush();

		if (serial.findUntil(">", "ERROR\r\n")) {

			serial.write(message);
			serial.write("\x1A\r\n");

			serial.setTimeout(10000);
			serial.flush();

			if (serial.findUntil("+CMGS: ", "ERROR\r\n")) {
			serial.setTimeout(1000);
				return true;
			}
		}
	}
	serial.setTimeout(1000);
	return false;
}

float GSM_G510::getSignalStrength() {
	serial.flush();
	serial.write("\r\n");
	delay(100);
	serial.write("AT+CSQ\r\n");
	serial.flush();
	
	return serial.parseFloat();
}


