#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdio.h>
#include <dht.h>
#include <string.h>
#include <SoftwareSerial.h>
#include "GSM_G510.h"
//GSM TX RX ON
GSM_G510 gsm = GSM_G510(10, 11, 12);
//Init DigitalHygroTempSensor
dht DHT;
#define DHT11_PIN 8
//Init OneWireBus
#define ONE_WIRE_BUS 9
//Init DallasSensors
#define MAX_DS1820_SENSORS 2
byte addr[MAX_DS1820_SENSORS][8];
//Init HD44780
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//OneWireBuffer
char buffer[25];
byte maxlines = 2;
char line1[16];
char line2[16];
byte maxsensors = 0;
OneWire  ds(ONE_WIRE_BUS);
DallasTemperature sensors(&ds);
boolean firstInitSMS = 0;
//Custom LCD Chars
byte hygro[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b10111,
  0b10011,
  0b10001,
  0b01110
};
byte t1[8] = {
  0b11100,
  0b01000,
  0b01000,
  0b00010,
  0b00110,
  0b00010,
  0b00010,
  0b00111
};
byte t2[8] = {
  0b11100,
  0b01000,
  0b01000,
  0b00110,
  0b00001,
  0b00011,
  0b00100,
  0b00111
};
byte t3[8] = {
  0b11100,
  0b01000,
  0b01000,
  0b00111,
  0b00001,
  0b00111,
  0b00001,
  0b00111
};
byte cels[8] = {
  0b01000,
  0b10100,
  0b01000,
  0b00000,
  0b00011,
  0b00100,
  0b00100,
  0b00011
};
void selfTest()
{
  lcd.clear();
  //DS18B20
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  sensors.requestTemperatures();
  while (ds.search(addr))
  {
    Serial.print("R=");
    for ( i = 0; i < 8; i++) {
      Serial.print(addr[i], HEX);
      Serial.print(" ");
    }
    if ( OneWire::crc8( addr, 7) != addr[7])
    {
      Serial.print("CRC is not valid!\n");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST1 - ");
      lcd.setCursor(0, 1);
      lcd.print("DS18B20:");
      lcd.setCursor(10, 1);
      lcd.print("CRCERR");
      delay(5000);
      lcd.clear();
      return;
    }
    {
      if (addr[0] == 0x28) {
        Serial.print("Device is a DS18B20 family device.\n");
        maxsensors++;
        lcd.setCursor(0, 0);
        lcd.print(" - SELF TEST1 - ");
        lcd.setCursor(0, 1);
        lcd.print("DS18B20:");
        lcd.setCursor(10, 1);
        lcd.print("[OK]");
        delay(1000);
        lcd.clear();
      }
      else {
        Serial.print("Device is unknown!\n");
        Serial.print("Device ID: ");
        Serial.print(addr[0], HEX);
        Serial.println();
        lcd.setCursor(0, 0);
        lcd.print(" - SELF TEST1 - ");
        lcd.setCursor(0, 1);
        lcd.print("Nie znane");
        lcd.setCursor(10, 1);
        lcd.print("[??]");
        delay(1000);
        lcd.clear();
        return;
      }

    }
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);
    Serial.print("P=");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print( OneWire::crc8( data, 8), HEX);
    Serial.println();
  }
  Serial.print("No more addresses.\n");
  lcd.clear();
  lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST1 - ");
      lcd.setCursor(0, 1);
      lcd.print("DS18B20:");
      lcd.setCursor(10, 1);
      lcd.print("AllAdr");
      delay(1000);
      lcd.clear();
  ds.reset_search();

  //DHT11
  Serial.print("DHT11, \t");
  int chk = DHT.read11(DHT11_PIN);
  switch (chk)
  {
    case DHTLIB_OK:
      Serial.print("OK,\t");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST2 - ");
      lcd.setCursor(0, 1);
      lcd.print("DHT11:");
      lcd.setCursor(10, 1);
      lcd.print("[OK]");
      delay(1000);
      lcd.clear();
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.print("Checksum error,\t");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST2 - ");
      lcd.setCursor(0, 1);
      lcd.print("DHT11:");
      lcd.setCursor(10, 1);
      lcd.print("[ckEr]");
      delay(1000);
      lcd.clear();
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.print("Time out error,\t");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST2 - ");
      lcd.setCursor(0, 1);
      lcd.print("DHT11:");
      lcd.setCursor(10, 1);
      lcd.print("[Tim?]");
      delay(1000);
      lcd.clear();
      break;
    case DHTLIB_ERROR_CONNECT:
      Serial.print("Connect error,\t");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST2 - ");
      lcd.setCursor(0, 1);
      lcd.print("DHT11:");
      lcd.setCursor(10, 1);
      lcd.print("[conE]");
      delay(1000);
      lcd.clear();
      break;
    case DHTLIB_ERROR_ACK_L:
      Serial.print("Ack Low error,\t");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST2 - ");
      lcd.setCursor(0, 1);
      lcd.print("DHT11:");
      lcd.setCursor(10, 1);
      lcd.print("[acLE]");
      delay(1000);
      lcd.clear();
      break;
    case DHTLIB_ERROR_ACK_H:
      Serial.print("Ack High error,\t");
      lcd.setCursor(0, 0);
      lcd.print(" - SELF TEST2 - ");
      lcd.setCursor(0, 1);
      lcd.print("DHT11:");
      lcd.setCursor(10, 1);
      lcd.print("[acHE]");
      delay(1000);
      lcd.clear();
      break;
    default:
      Serial.print("Unknown error,\t");
  }
  //GSM
  gsm.init();
  lcd.setCursor(0, 0);
  lcd.print(" - SELF TEST3 - ");
  lcd.setCursor(0, 1);
  lcd.print("Laczenie GSM... ");
  Serial.println("Start - oczekiwanie na polaczenie z siecia (15 sekund)");
  delay(19000);
  Serial.println("Wysylanie...");
  while (!gsm.sendSms("0048728480408", "PRZYWROCONO ZASILANIE NA MODULE GSM")) {
    Serial.println("GSM FAIL");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("Testuje siec:[?]");
        delay(5000);
      
   
  }
  Serial.println("GSM OK");
  while (1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("Sygnal asu: ");
    Serial.println((float)gsm.getSignalStrength(), 1);
    lcd.print((float)gsm.getSignalStrength(), 1);
    delay(1000);
    break;
    }
    //DataDe//lay
    delay(2000);
  }
  //Setup
  void setup()
  {
    //  gsm.init();
    lcd.begin(16, 2);
    lcd.print("Zespol Czujnikow");
    lcd.setCursor(0, 1);
    lcd.print("GSM  - v.02 2016");
    delay(2000);
    gsm.init();
    Serial.begin(9600);
    //do SelfTests
    selfTest();
    lcd.createChar(0, hygro);
    lcd.createChar(1, t1);
    lcd.createChar(2, t2);
    lcd.createChar(3, t3);
    lcd.createChar(4, cels);
    lcd.clear();
    //Reload Sensor Bus
    sensors.begin();
  }

  void loop()
  {
    lcd.clear();
    lcd.print("   Pomiary ... ");
    delay(500);
    ds18b20read();
    dht11read();
    delay(5000);
    waterRead();
    doorsensorRead();
    delay(5000);
    gsmStatus();
    delay(5000);
     lcd.clear();
    while (firstInitSMS == false)
      {
       
      firstInitSMS = 1;
      lcd.print("Wysylam SMS...");
      delay(1000);
      char sensor0[15];
      char sensor1[15];
      char sensorTDHT[15];
      char sensorHDHT[15];
/*    dtostrf(f0, 6, 2, sensor0);
      dtostrf(f1, 6, 2, sensor1);
      dtostrf((DHT.temperature, 1), 6, 2, sensorTDHT);
      dtostrf((DHT.humidity, 1), 6, 2, sensorHDHT);
      char sms[160];
        strcpy(sms, "Odczyt Czujnikow: ");
      strcat(sms, "\nTemp1: ");
      strcat(sms, sensor0);
      strcat(sms, "\nTemp2: ");
      strcat(sms, sensor1);
      strcat(sms, "\nDHT11 temp: ");
      strcat(sms, sensorTDHT);
      strcat(sms, "\nDHT11 hygro: ");
      strcat(sms, sensorHDHT);*/
      while (!gsm.sendSms("0048728480408", "PRZYWROCONO ZASILANIE NA MODULE GSM")) {
    Serial.println("GSM FAIL");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("Testuje siec:[?]");
        delay(5000);
      
   
  }
  Serial.println("GSM OK");
  while (1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("Sygnal asu: ");
    Serial.println((float)gsm.getSignalStrength(), 1);
    lcd.print((float)gsm.getSignalStrength(), 1);
    delay(1000);
    break;
    }

  }
  }
  /*void tempAlert(int dst1, dst2 ) //nie sprawdzone, nie dziala
    {
    lcd.clear();
        lcd.setCursor(0,10);
        lcd.print(dst1);
        lcd.setCursor(0,0);
        lcd.print("Alert:");
    }*/
  void dht11read()
  {
    int chk = DHT.read11(DHT11_PIN);
    Serial.print(",\t");
    lcd.setCursor(9,0);
    Serial.println(DHT.temperature, 1);
     lcd.write(byte(3));
      lcd.print(" ");
      lcd.print(DHT.temperature, 1);
      lcd.write(byte(4)); 
    Serial.print(DHT.humidity, 1);
    lcd.setCursor(9,1);
      lcd.write(byte(0));
      lcd.print(" ");
      lcd.print(DHT.humidity, 1);
      lcd.print("%");
  }

  void ds18b20read()
  {
    sensors.requestTemperatures();
    int ds1 = 0;
    int ds2 = 1;
    float f0 = sensors.getTempCByIndex(ds1);
    float f1 = sensors.getTempCByIndex(ds2);
    Serial.print("Sensor ");
    Serial.println(dtostrf(f1, 6, 2, buffer));
    Serial.println(dtostrf(f0, 6, 2, buffer));
    int temp0 = f0;
    int temp1 = f1;
    lcd.setCursor(0,0);
      lcd.clear();
      lcd.write(byte(1));
      lcd.print(dtostrf(f0, 6, 2, buffer));
      lcd.write(byte(4));
      lcd.setCursor(0,1);
      lcd.write(byte(2));
      lcd.print(dtostrf(f1, 6, 2, buffer));
      lcd.write(byte(4));
  }
  void waterRead()
  {
    //lcd.setCursor(0,0);
    //lcd.clear();
    // lcd.print("Woda:     [OK]");
  }
  void doorsensorRead()
  {
    //lcd.setCursor(0,1);
    //lcd.print("Drzwi:    [OK]");
  }
  void gsmStatus()
  {
    lcd.setCursor(0, 0);
    //    lcd.clear();
    // lcd.print((float)gsm.getIdent(),1);
    //    lcd.setCursor(0,1);
    //    lcd.print("Sygnal asu: ");
    //    lcd.print((float)gsm.getSignalStrength(),1);
  }

