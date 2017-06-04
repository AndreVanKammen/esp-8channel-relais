#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include "user_interface.h"
#include "i2s.h"

//#define SERIAL_DEBUG

#include "../../pwd/WIFI_AndreEnJantina.c"
const IPAddress mqtt_server = IPAddress(192, 168, 1, 20);

WiFiClient client;
PubSubClient mqtt(client);
long setupStartMillis;

int latchPin = 12;
int clockPin = 14;
int dataPin = 13;
int outputEnablePin = 4;

int keyBoardLine1 = 5;
int keyBoardLine2 = 3;
int keyBoardLine3 = 1;

char relaisState = 0;
char keyScanState = 0;

void checkForUpdate()
{
  t_httpUpdate_return ret = ESPhttpUpdate.update("http://192.168.1.20/espupdate/update.php", "relays.driver.bin"); // Should be the same as in uploadfirmware.txt
#ifdef SERIAL_DEBUG
  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
#endif
}

void setup()
{
#ifdef SERIAL_DEBUG
  setupStartMillis = millis();
  Serial.begin(74880);
  Serial.println();
  Serial.print("Setup start millis: ");
  Serial.println(setupStartMillis);
// Serial.print(", Mac: "); Serial.println(WiFi.macAddress());
// Mac: 5C:CF:7F:F8:77:06
#endif

  WiFi.mode(WIFI_STA);
  // Serial.print("Connecting to: "); Serial.println(ssid);

  WiFi.begin(ssid, password);
  WiFi.config(IPAddress(192, 168, 1, 132), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(outputEnablePin, OUTPUT);
  digitalWrite(outputEnablePin, HIGH);

  pinMode(keyBoardLine1, INPUT_PULLUP);
  pinMode(keyBoardLine2, INPUT_PULLUP);
  pinMode(keyBoardLine3, INPUT_PULLUP);

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(mqttMessage);
}

void UpdateShiftRegisters()
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, keyScanState ^ 0xFF);
  shiftOut(dataPin, clockPin, MSBFIRST, relaisState ^ 0xff);
  digitalWrite(latchPin, HIGH);
  digitalWrite(outputEnablePin, LOW);
}

void mqttMessage(char *topic, byte *payload, unsigned int length)
{
  relaisState = (char)payload[0];
  UpdateShiftRegisters();
#ifdef SERIAL_DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif
}

unsigned long reconnectTimeOut = 5000;
unsigned long lastConnectMillis = millis() - reconnectTimeOut;
void handleMqtt()
{
  unsigned long currentMillis = millis();
  if (mqtt.connected())
  {
    // Keep just in range
    if ((currentMillis - lastConnectMillis) > reconnectTimeOut)
      lastConnectMillis = currentMillis - reconnectTimeOut;
    mqtt.loop();
  }
  else
  {
    if ((currentMillis - lastConnectMillis) > reconnectTimeOut && (WiFi.status() == WL_CONNECTED))
    {
#ifdef SERIAL_DEBUG
      Serial.print("Attempting MQTT connection...");
#endif
      // Attempt to connect
      if (mqtt.connect("esp_8channel_relais"))
      {
#ifdef SERIAL_DEBUG
        Serial.print("Connected in: ");
        Serial.println(millis() - setupStartMillis);
        Serial.print("ms, IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(", Mac: ");
        Serial.print(WiFi.macAddress());
        Serial.println();
#endif
        // ... and resubscribe
        mqtt.subscribe("cmnd/relais/power");

        checkForUpdate();
      }
      else
      {
#ifdef SERIAL_DEBUG
        Serial.print("failed, rc=");
        Serial.print(mqtt.state());
        Serial.println(" try again in 5 seconds");
#endif
      }
      // Wait 5 seconds before retrying
      lastConnectMillis = currentMillis;
    }
  }
}

unsigned int keyStateCount[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool keyState[] = {false, false, false, false, false, false, false, false, false, false, false, false};
unsigned char keyboardMsg = 0;

char keyToChar(int keyNr)
{
  if (keyNr < 9)
  {
    return (char)keyNr + 49;
  }
  else
  {
    switch (keyNr)
    {
    case 9:
      return '*';
    case 10:
      return '0';
    case 11:
      return '#';
    }
  }
}

void updateKeyValue(int keyNr, int keySignal)
{
  if (keySignal == LOW)
  {
    if (keyStateCount[keyNr] < 8)
    {
      keyStateCount[keyNr] = keyStateCount[keyNr] + 1;
      if (keyStateCount[keyNr] == 5 && !keyState[keyNr])
      {
        keyState[keyNr] = true;
        keyboardMsg = keyToChar(keyNr);
        mqtt.publish("stat/keyboard/keydown", &keyboardMsg, 1);
        if (keyNr < 8)
          relaisState ^= 1 << keyNr;
        if (keyNr == 9)
          relaisState = 255;
        if (keyNr == 11)
          relaisState = 0;
      }
    }
  }
  else
  {
    if (keyStateCount[keyNr] > 0)
    {
      keyStateCount[keyNr]--;
      if (keyStateCount[keyNr] == 2 && keyState[keyNr])
      {
        keyState[keyNr] = false;

        keyboardMsg = keyToChar(keyNr);
        mqtt.publish("stat/keyboard/keyup", &keyboardMsg, 1);
      }
    }
  }
}

void updateKeyLine(int keyScan, int offset)
{
  keyScanState = keyScan;
  UpdateShiftRegisters();
  delay(1);
  updateKeyValue(offset + 0, digitalRead(keyBoardLine1));
  updateKeyValue(offset + 1, digitalRead(keyBoardLine2));
  updateKeyValue(offset + 2, digitalRead(keyBoardLine3));
}

void updateKeyState()
{
  updateKeyLine(8, 0);
  updateKeyLine(4, 3);
  updateKeyLine(2, 6);
  updateKeyLine(1, 9);
}

void loop()
{
  updateKeyState();
  handleMqtt();
  delay(1);
}
