/*
 * file HotTubDebug.ino
 * @ https://github.com/rdlauer/hot-tub-time-series-machine
 * 
 * Debugging a Hot Tub Time (Series) Machine
 * Follow the tutorial on Hackster here:
 * @ https://www.hackster.io/blues-wireless/projects
 * 
 * version v1.0
 * date 2021-11
 */

// #################################
// temp sensor constants and imports
// #################################

#include <OneWire.h>
int DS18S20_Pin = 6; // digital pin 6
OneWire ds(DS18S20_Pin);
float waterTemp = 0;

// ################################
// ORP sensor constants and imports
// ################################

#define VOLTAGE 5.00 // used for ORP and TDS sensors
#define OFFSET 0
#define sampleLength 40 // used for all sensors --> number of samples to acquire per result
#define orpPin 1        // analog pin 1
float orpValue;
int orpArray[sampleLength];
int orpArrayIndex = 0;

// ################################
// TDS sensor constants and imports
// ################################

#include <EEPROM.h>
#include "GravityTDS.h"
#define TdsSensorPin A2 // analog pin 2
GravityTDS gravityTds;
float tdsValue = 0;
int tdsArray[sampleLength];
int tdsArrayIndex = 0;

// ###############################
// ph sensor constants and imports
// ###############################

#include <DFRobot_PH.h>
//#include <EEPROM.h> (already included above, saving this for reference)
#define PH_PIN A3 // analog pin 3
float phValue = 0, phVoltage;
int phArray[sampleLength];
int phArrayIndex = 0;
DFRobot_PH ph;

// ##############################
// notecard constants and imports
// ##############################

#include <Notecard.h>
#define productUID "[notehub.io product identifier]"
Notecard notecard;
float ambientTemp = 0;

void setup()
{
  Serial.begin(9600);

  // TDS sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(VOLTAGE);
  gravityTds.setAdcRange(1024); // 1024 for 10bit ADC; 4096 for 12bit ADC
  gravityTds.begin();

  // ph sensor
  ph.begin();

  // notecard: associate w/ notehub project, set sync freq based on battery type & voltage
  notecard.begin();

  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", productUID);
  JAddStringToObject(req, "mode", "periodic");
  JAddStringToObject(req, "voutbound", "usb:30;high:60;normal:120;low:240;dead:0");
  JAddStringToObject(req, "vinbound", "usb:30;high:360;normal:720;low:1440;dead:0");
  notecard.sendRequest(req);

  req = notecard.newRequest("card.voltage");
  JAddStringToObject(req, "mode", "lipo");
  notecard.sendRequest(req);
}

void loop()
{

  // #######################
  // gather temp sensor data
  // #######################

  waterTemp = getWaterTemp();
  Serial.print("\nwater: ");
  Serial.print(waterTemp);
  Serial.println("C");

  ambientTemp = getAmbientTemp();
  Serial.print("air: ");
  Serial.print(ambientTemp);
  Serial.println("C");

  // get 40 samples of each measurement, separated by 100ms
  for (int i = 0; i < sampleLength; i++)
  {

    // ######################
    // gather ORP sensor data
    // ######################

    orpArray[orpArrayIndex++] = analogRead(orpPin);

    if (orpArrayIndex == sampleLength)
    {
      orpValue = ((30 * (double)VOLTAGE * 1000) - (75 * averageArray(orpArray, sampleLength) * VOLTAGE * 1000 / 1024)) / 75 - OFFSET;
      Serial.print("ORP: ");
      Serial.print(orpValue);
      Serial.println("mV");
      orpArrayIndex = 0;
    }

    // ######################
    // gather TDS sensor data
    // ######################

    gravityTds.setTemperature(waterTemp);
    gravityTds.update();

    tdsArray[tdsArrayIndex++] = gravityTds.getTdsValue();

    if (tdsArrayIndex == sampleLength)
    {
      tdsValue = averageArray(tdsArray, sampleLength);
      Serial.print("TDS: ");
      Serial.print(tdsValue);
      Serial.println("ppm");
      tdsArrayIndex = 0;
    }

    // #####################
    // gather ph sensor data
    // #####################

    phVoltage = analogRead(PH_PIN) / 1024.0 * 5000;
    phArray[phArrayIndex++] = ph.readPH(phVoltage, waterTemp);

    if (phArrayIndex == sampleLength)
    {
      phValue = averageArray(phArray, sampleLength);
      Serial.print("ph: ");
      Serial.print(phValue);
      phArrayIndex = 0;
    }

    delay(100);
  }

  // send accumulated data to the cloud!
  sendToCloud(ambientTemp, waterTemp, orpValue, tdsValue, phValue);

  // 30 min delay between new readings
  delay(1000 * 60 * 30);
}

float getAmbientTemp()
{
  // returns the ambient temp as measured by the temp sensor on the notecard
  float temp = 0;
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.temp"));
  if (rsp != NULL)
  {
    temp = JGetNumber(rsp, "value");
    notecard.deleteResponse(rsp);
  }
  return temp;
}

float getWaterTemp()
{
  // returns the water temp from the DS18S20 sensor
  byte data[12];
  byte addr[8];

  if (!ds.search(addr))
  {
    Serial.println("no more sensors on chain, reset search!");
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB);
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

void sendToCloud(float ambientTemp, float waterTemp, float orpValue, float tdsValue, float phValue)
{

  // ##############################################################
  // get the current battery voltage so we can add that to the data
  // ##############################################################

  float voltage = 0;
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.voltage"));
  JAddStringToObject(rsp, "mode", "?");
  if (rsp != NULL)
  {
    voltage = JGetNumber(rsp, "value");
    notecard.deleteResponse(rsp);
  }

  // ############################################
  // send gathered data to notehub for processing
  // ############################################

  J *req = notecard.newRequest("note.add");
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "hottub.qo");
    J *body = JCreateObject();
    if (body != NULL)
    {
      JAddNumberToObject(body, "ambient_temp", ambientTemp);
      JAddNumberToObject(body, "water_temp", waterTemp);
      JAddNumberToObject(body, "orp_value", orpValue);
      JAddNumberToObject(body, "tds_value", tdsValue);
      JAddNumberToObject(body, "ph_value", phValue);
      JAddNumberToObject(body, "voltage", voltage);
      JAddItemToObject(req, "body", body);
    }
    notecard.sendRequest(req);
  }

  // ###################################################
  // send a text message if anything is WAY out of range
  // ###################################################

  if ((orpValue != NULL && orpValue < 650) || (tdsValue != NULL && tdsValue > 1500) || (phValue != NULL && (phValue < 7 || phValue > 8)))
  {
    req = notecard.newRequest("note.add");
    if (req != NULL)
    {
      JAddStringToObject(req, "file", "hottub_notify.qo");
      JAddBoolToObject(req, "sync", true);
      J *body = JCreateObject();
      if (body != NULL)
      {
        JAddNumberToObject(body, "orp_value", orpValue);
        JAddNumberToObject(body, "tds_value", tdsValue);
        JAddNumberToObject(body, "ph_value", phValue);
        JAddStringToObject(body, "from", "[twilio number]");
        JAddStringToObject(body, "to", "[to number]");
        JAddItemToObject(req, "body", body);
      }
      notecard.sendRequest(req);
    }
  }
}

double averageArray(int *arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0)
  {
    return 0;
  }
  if (number < 5)
  {
    for (i = 0; i < number; i++)
    {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }
  else
  {
    if (arr[0] < arr[1])
    {
      min = arr[0];
      max = arr[1];
    }
    else
    {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++)
    {
      if (arr[i] < min)
      {
        amount += min;
        min = arr[i];
      }
      else
      {
        if (arr[i] > max)
        {
          amount += max;
          max = arr[i];
        }
        else
        {
          amount += arr[i];
        }
      }
    }
    avg = (double)amount / (number - 2);
  }
  return avg;
}
