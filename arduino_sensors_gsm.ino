//TestChange
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

/////////////////////////
//ALERTS//SETUP//VALUES//
const float temp1Hi = 50;
const float temp1Lo = 5;
const float temp2Hi = 45;
const float temp2Lo = 5;
const double temp3Hi = 35;
const double temp3Lo = 5;
const double hygroHi = 50;
/////////////////////////
char smsnumber[13];
char sms[160];
/////////////////////////

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
//WaterSensor
const int WaterRead = A7;
//DoorSensor
const int DoorRead = A6;
//Init HD44780
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//OneWireBuffer
char buffer[25];
char bufdht[25];
byte maxsensors = 0;
OneWire  ds(ONE_WIRE_BUS);
DallasTemperature sensors(&ds);
boolean firstInitSMS = 0;
boolean isAlert = 0;
bool wyslanoSMS = false;
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
byte antenna[8] = {
  0b00000,
  0b10101,
  0b01110,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};
void selfTest()
{
  lcd.clear();
  gsm.init();
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
  unsigned int smsattemp = 0;
  lcd.setCursor(0, 0);
  lcd.print(" - SELF TEST3 - ");
  lcd.setCursor(0, 1);
  lcd.print("Laczenie GSM... ");
  Serial.println("Start - oczekiwanie na polaczenie z siecia (15 sekund)");
  delay(15000);
  Serial.println("Wysylanie...");
  while (!gsm.sendSms(smsnumber, "PRZYWROCONO ZASILANIE NA MODULE GSM")) {
    Serial.println("GSM - oczekuje...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("[Czekam: ");
    lcd.print(smsattemp);
    lcd.print("]");
    delay(1000);
    if (smsattemp == 5)
      break;
    smsattemp++;
  }
  if (smsattemp == 5) {
    Serial.println("GSM Fail");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("Blad Polaczenia!");
    delay(5000);
  }
  else {
    Serial.println("GSM OK");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" - SELF TEST3 - ");
    lcd.setCursor(0, 1);
    lcd.print("Sygnal asu: ");
    Serial.println((float)gsm.getSignalStrength(), 1);
    lcd.print((float)gsm.getSignalStrength(), 1);
    delay(1000);
  }
}
//Setup
void setup()
{
  strcpy(smsnumber, "0048728480408");
  pinMode(13, OUTPUT);
  lcd.begin(16, 2);
  lcd.print("   Sensor-BOX   ");
  lcd.setCursor(0, 1);
  delay(3000);
  lcd.print("  rev.1.02-GSM" );
  delay(1500);
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("L.Skwarszczow");
  lcd.setCursor(0, 1);
  lcd.print("dla: IWKD.net ");
  delay(1500);
  Serial.begin(9600);
  //do SelfTests
  selfTest();
  lcd.createChar(0, hygro);
  lcd.createChar(1, t1);
  lcd.createChar(2, t2);
  lcd.createChar(3, t3);
  lcd.createChar(4, cels);
  lcd.createChar(5, antenna);
  lcd.clear();
  //Reload Sensor Bus
  sensors.begin();
}

void loop()
{
  char sms[160];
  sensorsToDataStream();

}

void sensorsToDataStream()
{
  Serial.println("POMIARY... \t");
  digitalWrite(13, LOW);
  float f0, f1;
  int WaterLevel;
  double dht11hygro, dht11temp;
  bool doorStatus;
  bool waterStatus;
  char doorStatus0[10];
  char waterStatus0[10];
  char AlertMessageSign[25];
  int doorResistance;
  ds18b20read(f0, f1);
  dht11read(dht11hygro, dht11temp);
  delay(6000);
  lcd.clear();
  WaterSensor(WaterLevel);
  doorsensorRead(doorResistance, doorStatus);
  delay(1500);
  gsmStatus();
  delay(1500);
  ///////////////////
bool alert1temp = false;
  if (f0 <= temp1Lo || f0 >= temp1Hi) {
    alert1temp = true;
    strcpy(AlertMessageSign, "ALERT - TEMP1, ");
  }
  else {
    alert1temp = false;
  }
  //Temp 2 alert
  bool alert2temp = false;
  if (f1 <= temp2Lo || f1 >= temp2Hi) {
    alert2temp = true;
    strcpy(AlertMessageSign, "ALERT - TEMP2, ");
  }
  else {
    alert2temp = false;
  }
  //Temp 3 alert
  bool alert3temp = false;
  if (dht11temp <= temp3Lo || dht11temp >= temp3Hi) {
    alert3temp = true;
    strcpy(AlertMessageSign, "ALERT - TEMP3, ");
  }
  else {
    alert3temp = false;
  }
  //Hygro Alert
  bool alert4hygro = false;
  if (dht11hygro >= hygroHi) {
    alert4hygro = true;
    strcpy(AlertMessageSign, "ALERT - WILGOTNOSC, ");
  }
  else {
    alert4hygro = false;
  }
  //Door Alert
  //doorStatus(bool->char)
  if (doorStatus == false) {
    strcpy(doorStatus0, "OTWARTE!");
    strcpy(AlertMessageSign, "ALERT CZUJNIKA DRZWI, ");
  }
  else {
    strcpy(doorStatus0, "Zamkniete");
  }
  //Water Alert
  if (WaterLevel < 150) {
    strcpy(doorStatus0, "OK");
    waterStatus = false;
  }
  else {
    strcpy(doorStatus0, "ALERT!");
    strcpy(AlertMessageSign, "ALERT CZUJNIKA WODY, ");
    waterStatus = true;
  }

  ///////////////////
  char sensor0[15];
      char sensor1[15];
      char sensorTDHT[15];
      char sensorHDHT[15];
      char waterSMS[10];
      char doorSMS[10];
      dtostrf(f0, 6, 2, sensor0);
      dtostrf(f1, 6, 2, sensor1);
      dtostrf(dht11temp, 6, 2, sensorTDHT);
      dtostrf(dht11hygro, 6, 2, sensorHDHT);
      strcpy(sms, "Odczyt Czujnikow: ");
      strcat(sms, AlertMessageSign);
      strcat(sms, "\nTemp1: ");
      strcat(sms, sensor0);
      strcat(sms, "\nTemp2: ");
      strcat(sms, sensor1);
      strcat(sms, "\nDHT11 temp: ");
      strcat(sms, sensorTDHT);
      strcat(sms, "\nDHT11 hygro: ");
      strcat(sms, sensorHDHT);
      strcat(sms, "\nDrzwi: ");
      strcat(sms, doorStatus0);
      strcat(sms, "\nWoda: ");
      strcat(sms, waterStatus0);
}
void sendMessage()
{
  unsigned int sms2attemp = 0;
  while (!gsm.sendSms(smsnumber, sms)) {
    Serial.println("Wysylanie SMS");
    lcd.setCursor(0, 1);
    lcd.print("[Czekam: ");
    lcd.print(sms2attemp);
    lcd.print("]");
    delay(1000);
    if (sms2attemp == 5)
      break;
    sms2attemp++;
  }
  if (sms2attemp == 20) {
    Serial.println("GSM Fail");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GSM ALERT");
    lcd.setCursor(0, 1);
    lcd.print("Blad Polaczenia!");
    delay(5000);
  }
  else {
    Serial.println("GSM OK");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Wiadomosc SMS");
    lcd.setCursor(0, 1);
    lcd.print("Wyslana!");
    delay(1000);
  }
}
void dht11read(double &dht11hygro, double &dht11temp)
{
  int chk = DHT.read11(DHT11_PIN);
  Serial.println("DHT11 \t");
  lcd.setCursor(9, 0);
  dht11temp = (DHT.temperature);
  Serial.println("Temperatura DHT: ");
  Serial.println(dht11temp, 0);
  lcd.write(byte(3));
  lcd.print(" ");
  lcd.print(dht11temp, 0);
  lcd.write(byte(4));
  dht11hygro = (DHT.humidity);
  Serial.println("Wilgotnosc DHT: ");
  Serial.println(dht11hygro, 0);
  lcd.setCursor(9, 1);
  lcd.write(byte(0));
  lcd.print(" ");
  lcd.print(dht11hygro, 0);
  lcd.print("%");
}
void ds18b20read(float &f0, float &f1)
{
  sensors.requestTemperatures();
  int ds1 = 0;
  int ds2 = 1;
  float dstemp0, dstemp1;
  f0 = sensors.getTempCByIndex(ds1);
  f1 = sensors.getTempCByIndex(ds2);
  Serial.println("DS18b20 \t");
  Serial.println("DS 1 temp: ");
  Serial.println(dtostrf(f1, 6, 2, buffer));
  Serial.println("DS 2 temp: ");
  Serial.println(dtostrf(f0, 6, 2, buffer));
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.write(byte(1));
  lcd.print(" ");
  lcd.print(f0);
  lcd.write(byte(4));
  lcd.setCursor(0, 1);
  lcd.write(byte(2));
  lcd.print(" ");
  lcd.print(f1);
  lcd.write(byte(4));
}

void doorsensorRead(int &doorResistance, bool &doorStatus)
{
  doorResistance = analogRead(DoorRead);
  Serial.println("Drzwi [OK <= 15]:");
  Serial.println(doorResistance);
  lcd.setCursor(0, 1);
  lcd.print("Drzwi:");
  if (doorResistance < 15) {
    lcd.setCursor(8, 1);
    lcd.print("[OK]");
    doorStatus = true;
  }
  else {
    lcd.setCursor(8, 1);
    lcd.print("[ALERT!]");
    doorStatus = false;
    digitalWrite(13, HIGH);
  }

}
void gsmStatus()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GSM Status");
  lcd.setCursor(0, 1);
  lcd.write(byte(5));
  lcd.print(" ");
  lcd.print((float)gsm.getSignalStrength(), 1);
  lcd.print(" asu");
  Serial.println("Sygnal GSM: ");
  Serial.println((float)gsm.getSignalStrength(), 1);
}

void WaterSensor(int &WaterLevel)
{
  WaterLevel = analogRead(WaterRead);
  Serial.println("Woda [OK <= 150]:");
  Serial.println(WaterLevel);
  lcd.setCursor(0, 0);
  lcd.print("Woda:");
  if (WaterLevel < 150) {
    lcd.setCursor(8, 0);
    lcd.print("[OK]");
  }
  else {
    lcd.setCursor(8, 0);
    lcd.print("[ALERT!]");
    digitalWrite(13, HIGH);
  }

}
