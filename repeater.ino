#include <PubSubClient.h>
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include "Adafruit_SHT4x.h"
#include <otadrive_esp.h>

//#define GSM_REPEATER

#ifdef GSM_REPEATER
// REPETARE GSM-------------------------------------------
#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>
#include <time.h>
#include <WiFi.h>

// Define serial
// #define SerialMon Serial
#define SerialAT Serial2
// #define DEBUG_AT_COMMANDS

// RX, TX fot UART GSM
#define PIN_TX 17
#define PIN_RX 16

// Range to attempt to autobaud
// NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// communication, set a fixed baud rate using modem.setBaud(#).
#define GSM_BAUD 19200
#define GSM_PIN ""
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Define how you're planning to connect to the internet.
// This is only needed for this example, not in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// Define variables

// Your GPRS credentials, if any
const char apn[] = "plus";
const char gprsUser[] = "";
const char gprsPass[] = "";

// SNTP time config
const char *NTP_SERVER = "us.pool.ntp.org";
const unsigned long NTP_UPDATE_INTERVAL = 1800000; // ms between NTP queries

#ifdef DEBUG_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Initializations of network clients
TinyGsmClient client(modem);
PubSubClient mqttClient(client);
TinyGsmClient gsm_otadrive_client(modem, 1);
unsigned long actualTime;

#else
// REPETARE POE-------------------------------------------
#include <ETH.h>
#include <WiFiClient.h>
#include <NTPClient.h>
unsigned long actualTime;

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN 12
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR 0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN 23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN 18
// SNTP time config

const char *NTP_SERVER = "us.pool.ntp.org";
const unsigned long NTP_UPDATE_INTERVAL = 1800000; // ms between NTP queries
static bool eth_connected = false;

// Initializations of network clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, 0, NTP_UPDATE_INTERVAL);
#endif

bool ShtConnected = false;

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

typedef struct
{
  unsigned long scannedTime;
  std::string address;
  uint8_t advData[50];
  uint8_t name[12];
  int nameLength;
  int rssi;
  int advDataLength;
} record_sensor;

// SCAN TIME
#define SCAN_TIME 3 // In seconds
#define NUMBER_OF_SCANNED_SENSOR 300
#define DELAY_TIME 2 // In secnods

// LED
#define MLED 3

struct tm timeinfo;
long scannedDevice = 0;
uint32_t lastReconnectAttempt = 0;
record_sensor ScannedSensors[NUMBER_OF_SCANNED_SENSOR];
static String mac = "";

// CHANGE THESE SETTINGS FOR YOUR APPLICATION
const char *mqttServerIp = "168.63.59.193"; // IP address of the MQTT broker
const short mqttServerPort = 1883;          // IP port of the MQTT broker
const char *mqttClientName = "ESP32-POE";
const char *mqttUsername = "skk-mqtt";
const char *mqttPassword = "Mqtt.rnd123";

NimBLEScan *pBLEScan;

#ifdef GSM_REPEATER
unsigned long getGsmTime()
{
  struct tm t;
  time_t t_of_day;
  int year3 = 0;
  int month3 = 0;
  int day3 = 0;
  int hour3 = 0;
  int min3 = 0;
  int sec3 = 0;
  float timezone = 0;
  for (int8_t i = 5; i; i--)
  {
    Serial.print("Requesting current network time... ");
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3, &timezone))
      break;

    Serial.println("Couldn't get network time, retrying in 15s.");
  }
  /*DBG("Retrieving time again as a string");
  String timeNow = modem.getGSMDateTime(DATE_TIME);
  Serial.print("Current Network Time: ");
  Serial.println(timeNow);*/
  Serial.print("Success: ");
  Serial.print(year3 - 1900);
  Serial.print("-");
  Serial.print(month3);
  Serial.print("-");
  Serial.print(day3);
  Serial.print(" ");
  Serial.print(hour3);
  Serial.print(":");
  Serial.print(min3);
  Serial.print(":");
  Serial.print(sec3);
  Serial.print("'");
  Serial.print(timezone);
  t.tm_year = year3 - 1900; // Year - 1900
  t.tm_mon = month3 - 1;    // Month, where 0 = jan
  t.tm_mday = day3;         // Day of the month
  t.tm_hour = hour3;
  t.tm_min = min3;
  t.tm_sec = sec3;
  if (timezone > 0)
    t.tm_hour -= timezone;
  else if (timezone < 0)
    t.tm_hour += timezone;

  t.tm_isdst = -1; // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);
  Serial.print(";Long: ");

  Serial.print((unsigned long)t_of_day);
  return (unsigned long)t_of_day;
}
#endif

String getIsoTime(unsigned long epochTime)
{
  char timeStr[21] = {0}; // NOTE: change if strftime format changes

  time_t time_now = epochTime;
  localtime_r(&time_now, &timeinfo);

  if (timeinfo.tm_year <= (2016 - 1900))
  {
    return String("YYYY-MM-DDTHH:MM:SSZ");
  }
  else
  {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    // return timeStr;
    return String(timeStr);
  }
}

String hexToStr(uint8_t *arr, int n, bool isMac)
{
  String result;
  if (isMac)
    result.reserve(2 * n + n - 1);
  else
    result.reserve(2 * n);
  for (int i = 0; i < n; ++i)
  {
    if (arr[i] < 0x10)
      result += '0';
    result += String(arr[i], HEX);
    if (i < n - 1 && isMac)
      result += ":";
  }

  return result;
}

uint8_t oneByteTemp(float temp)
{
  uint16_t St = (uint16_t)(374 * temp + 16852);
  uint8_t Ktemp = (-4800000 + 854 * St) / 100000;
  return Ktemp;
}
uint8_t oneByteHum(float hum)
{
  uint16_t RH = (uint16_t)(524 * hum + 3144);
  uint8_t Khum = (256 + (391 * RH)) / 100000;
  return Khum;
}

void AddRepeaterSensorDataToScannedList()
{

  sensors_event_t humidity, temp_event;
  sht4.getEvent(&humidity, &temp_event);

  record_sensor repeater;

  String(repeater.scannedTime) = String(getIsoTime(actualTime));
  repeater.address = WiFi.macAddress().c_str();
  repeater.rssi = -10;
  repeater.advData[0] = 0x02;
  repeater.advData[1] = 0x01;
  repeater.advData[2] = 0x06;
  repeater.advData[3] = 0x0E;
  repeater.advData[4] = 0xFF;
  repeater.advData[5] = 0x02;
  repeater.advData[6] = 0x0D;
  repeater.advData[7] = 0xFF;
  repeater.advData[8] = 0xC0;
  repeater.advData[9] = 0x00;
  repeater.advData[10] = 0xC0;
  repeater.advData[11] = 0x00;
  repeater.advData[12] = oneByteTemp(temp_event.temperature);
  repeater.advData[13] = oneByteHum(humidity.relative_humidity);

  ScannedSensors[scannedDevice++] = repeater;
}

uint8_t crc_calc(const char *data, uint8_t count)
{
  uint16_t current_byte;
  uint8_t crc = 0xFF;         // CRC8_INIT
  uint8_t polynominal = 0x31; // CRC8_POLYNOMIAL
  uint8_t crc_bit;

  /* calculates 8-Bit checksum with given polynomial */
  for (current_byte = 0; current_byte < count; ++current_byte)
  {
    crc ^= (data[current_byte]);
    for (crc_bit = 8; crc_bit > 0; --crc_bit)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ polynominal;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

void SendBleScanResults()
{
  if (ShtConnected)
  {
    AddRepeaterSensorDataToScannedList();
  }
  digitalWrite(MLED, HIGH);
  for (int i = 0; i < scannedDevice; i++)
  {

    // BLEAdvertisedDevice advertisedDevice = result.getDevice(i);
    Serial.print(getIsoTime(ScannedSensors[i].scannedTime));
    Serial.print(", ");
    Serial.print(ScannedSensors[i].address.c_str());

    int rssi = 0;
    Serial.print(", ");
    rssi = ScannedSensors[i].rssi;
    Serial.print(rssi);

    Serial.print(", ");
    String payload = hexToStr(ScannedSensors[i].advData, ScannedSensors[i].advDataLength, false);
    Serial.print(payload);
    Serial.print(", ");

    /* Read name from payload */
    String nameCharArray = (char *)ScannedSensors[i].name;
    String nameStringArray = nameCharArray.substring(0, ScannedSensors[i].nameLength);
    Serial.print(nameStringArray);
    Serial.print(", ");

    // CRC
    String dataToCRC = String(getIsoTime(ScannedSensors[i].scannedTime) + ";" + nameStringArray + ";" + ScannedSensors[i].address.c_str() + ";" + String(rssi) + ";" + payload + ";");

    uint8_t crcValue = crc_calc(dataToCRC.c_str(), dataToCRC.length());

    Serial.print(crcValue);
    Serial.print(", ");

    String dataToSend = String(getIsoTime(ScannedSensors[i].scannedTime) + ";" + nameStringArray + ";" + ScannedSensors[i].address.c_str() + ";" + String(rssi) + ";" + payload + ";" + crcValue);
    Serial.print("Data to send is ready");
    // Serial.printf("%s, Rssi: %d \n", advertisedDevice.toString().c_str(), rssi);
#ifndef GSM_REPEATER
    mac = String(ETH.macAddress());
#else
    mac = String(WiFi.macAddress());
#endif
    String topic = String(mac + "/ble");
    Serial.print(": Published");
    if (mqttClient.connected())
    {
      mqttClient.publish(topic.c_str(), dataToSend.c_str());
      Serial.println("Done");
    }
    else
    {
      Serial.println("MQTT - no connection");
      reconnect();
      mqttClient.publish(topic.c_str(), dataToSend.c_str());
    }
  }
  digitalWrite(MLED, LOW);
}

class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *advertisedDevice)
  {
    digitalWrite(MLED, HIGH);
    int advLength = advertisedDevice->getPayloadLength() - advertisedDevice->getName().length();
    if (scannedDevice < NUMBER_OF_SCANNED_SENSOR)
    {
      if (advertisedDevice->haveRSSI() == false || advLength < 20 || advLength > 28)
        return;
      Serial.print("New Device: ");
      Serial.println(String(advertisedDevice->getAddress().toString().c_str()));

#ifndef GSM_REPEATER
      ScannedSensors[scannedDevice].scannedTime = ntpClient.getEpochTime();
#else
      ScannedSensors[scannedDevice].scannedTime = actualTime;
#endif
      ScannedSensors[scannedDevice].rssi = advertisedDevice->getRSSI();
      ScannedSensors[scannedDevice].address = advertisedDevice->getAddress().toString().c_str();
      memcpy(ScannedSensors[scannedDevice].advData, advertisedDevice->getPayload(), advLength);

      int nameLength = advertisedDevice->getName().length();
      if (nameLength > 12)
      {
        nameLength = 12;
      }

      memcpy(ScannedSensors[scannedDevice].name, advertisedDevice->getName().c_str(), nameLength);
      ScannedSensors[scannedDevice].nameLength = nameLength;
      ScannedSensors[scannedDevice].advDataLength = advLength;
      scannedDevice++;
    }

    digitalWrite(MLED, LOW);
  }
};

bool reconnect()
{
  // Loop until we're reconnected
  for (int i = 0; i < 5; i++)
  {
    if (mqttClient.connected())
      return true;

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(mqttClientName, mqttUsername, mqttPassword))
    {
      Serial.println("connected");
      String statusString = mac + String("/status");
      mqttClient.publish(statusString.c_str(), "ping");
      return true;
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
  return false;
}

#ifndef GSM_REPEATER
void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_ETH_START:
    Serial.println("ETH Started");
    // set eth hostname here
    ETH.setHostname("esp32-ethernet");
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println("");
    Serial.println("ETH Connected");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex())
    {
      Serial.print(", FULL_DUPLEX");
    }
    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}
#endif

#ifdef GSM_REPEATER
void InitGSM()
{
  delay(10);
  Serial2.begin(GSM_BAUD, SERIAL_8N1, PIN_RX, PIN_TX, false);
  delay(100);

  Serial.println("Wait...");
  TinyGsmAutoBaud(Serial2, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isNetworkConnected())
  {
    Serial.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isGprsConnected())
  {
    Serial.println("GPRS connected");
  }
#endif
}
#endif

void SHT_Init()
{
  // Check if SHT40 is working
  for (uint8_t i = 0; i < 3; i++)
  {
    if (!sht4.begin())
    {
      Serial.println("Couldn't find SHT4x");
      ShtConnected = false;
    }
    else
    {
      ShtConnected = true;
      Serial.println("Successfull connected to SHT sensor");
      Serial.println(sht4.readSerial(), HEX);
      break;
    }
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    delay(1000);
  }
}
#ifdef GSM_REPEATER
void update_prgs(size_t i, size_t total){
    Serial.print("upgrade ");
    Serial.print(i);
    Serial.print("/");
    Serial.print(total);
    Serial.print(" ");
    Serial.print((i * 100) / total);
    Serial.println("%");
}
#else
void ota()
{
  if(OTADRIVE.timeTick(30))
  {
    OTADRIVE.updateFirmware();
  }
}
#endif
void setup()
{
  Serial.begin(115200);
  // SETUP TIMER FOR LED CHANGE
  pinMode(MLED, OUTPUT);
  digitalWrite(MLED, LOW);

  SHT_Init();

#ifdef GSM_REPEATER
  InitGSM();
  // initialize option to remonte update device
  OTADRIVE.setInfo("f093ef7f-7d13-4fc7-af64-2c448928b852", "1.1.0");
  SPIFFS.begin(true);
  OTADRIVE.onUpdateFirmwareProgress(update_prgs);
  Serial.print("Download a new firmware from SIM7000, V=");
  Serial.println(OTADRIVE.Version.c_str());

#else
  WiFi.onEvent(WiFiEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

  Serial.println(ETH.macAddress());
  if (!eth_connected)
  {
    Serial.print("ETH connection");
    for (uint8_t i = 0; i < 100; i++)
    {
      if (eth_connected == true)
        break;

      Serial.print(".");
      delay(1000);
    }
  }
  OTADRIVE.setInfo("f093ef7f-7d13-4fc7-af64-2c448928b852", "v@1.1.1");
#endif



  Serial.println("Initialize Bluettooth");
  NimBLEDevice::init("Repeater Device"); // Repeater Device
  pBLEScan = NimBLEDevice::getScan();    // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
  pBLEScan->setActiveScan(true); // active scan uses more power, but get results faster
  pBLEScan->setInterval(200);
  pBLEScan->setWindow(100);   // less or equal setInterval value
  pBLEScan->setMaxResults(0); // do not store the scan results, use callback only.
  mqttClient.setServer(mqttServerIp, mqttServerPort);

#ifndef GSM_REPEATER
  ntpClient.begin();
  ntpClient.update();
#endif
}

void loop()
{

  Serial.print("Check Connection.");

#ifdef GSM_REPEATER
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected())
  {
    Serial.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true))
    {
      Serial.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected())
    {
      Serial.println("Network re-connected");
    }

#if TINY_GSM_USE_GPRS
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected())
    {
      Serial.println("GPRS disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass))
      {
        Serial.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected())
      {
        Serial.println("GPRS reconnected");
      }
    }
#endif
  }
#endif

#ifdef GSM_REPEATER
  if(!OTADRIVE.timeTick(30)){
    delay(3000);
    return;
  }
    if(modem.isNetworkConnected()){
      Serial.println("Connected to network...");
      OTADRIVE.updateFirmware(gsm_otadrive_client);
    }
#else
  ota();
#endif


  Serial.print("Connection Ok. Check MQTT.");
  if (!mqttClient.connected())
  {
    Serial.println("=== MQTT NOT CONNECTED ==="); // SerialMon
    if (reconnect())
      mqttClient.loop();
  }

#ifndef GSM_REPEATER
  Serial.println("");
  Serial.println("NTP Update");
  ntpClient.update();
#endif

  if (mqttClient.connected())
  {
    Serial.println("Start scan");
    scannedDevice = 0;
#ifdef GSM_REPEATER
    actualTime = getGsmTime();
#endif
    NimBLEScanResults foundDevices = pBLEScan->start(SCAN_TIME, false);
    Serial.println("");
    Serial.print("End scan. Founded device: ");
    Serial.println(scannedDevice);
    String statusString = mac + String("/status");
    mqttClient.publish(statusString.c_str(), "ping");
    SendBleScanResults();
#ifdef GSM_REPEATER
    actualTime = 0;
#endif
    pBLEScan->clearDuplicateCache();
    pBLEScan->clearResults();
  }

 delay((SCAN_TIME + DELAY_TIME) * 1000);

}
