#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <FS.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESPAsyncWebServer.h>
#include <Ticker.h>
#include <Wire.h>
#include "MPU9250.h"

#define INTERRUPT_PIN 18
#define LED 19

// from quaternionFilters.cpp
void MadgwickQuaternionUpdate(float q[], uint32_t interval, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float q[], uint32_t interval, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

Ticker ticker;

uint64_t chipid = ESP.getEfuseMac();
char hostName[32];

long unsigned int now, last, offsetTime;
uint32_t updateDelay = 50;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

StaticJsonDocument<1024> jsonDoc;
JsonObject rootTx = jsonDoc.to<JsonObject>();

MPU9250FIFO IMU(Wire, 0x68);
int status;

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
boolean madgwick = false;

boolean newData = false;
void IRAM_ATTR getIMU() {
  newData = true;
}

void tick() {
  //toggle state
  digitalWrite(LED, !digitalRead(LED));     // set pin to the opposite state
}

void flash() {
  digitalWrite(LED, HIGH);
  ticker.once(0.5, tick);
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u:)", client->id());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT) {
    Serial.printf("ws[%s][%u] disconnect.\n", server->url(), client->id());
  } else if(type == WS_EVT_ERROR) {
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_PONG) {
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA) {
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if(info->final && info->index == 0 && info->len == len) {
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

      if(info->opcode == WS_TEXT) {
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n", msg.c_str());

      if(info->opcode == WS_TEXT) {
        client->text("I got your text message");
      } else {
        client->binary("I got your binary message");
      }
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0) {
        if(info->num == 0) {
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
        }
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

      if(info->opcode == WS_TEXT) {
        for(size_t i=0; i < len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if((info->index + len) == info->len) {
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if(info->final) {
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text"  :"binary");
          if(info->message_opcode == WS_TEXT) {
            client->text("I got your text message");
          } else {
            client->binary("I got your binary message");
          }
        }
      }
    }
  }
}

void initIMU() {
  Serial.println("Initialize IMU");
  status = IMU.begin();
  if (status < 0) {
    Serial.println("\nIMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.printf("Status: %d\n", status);
  } else {
    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 9 for a 100 Hz update rate
    IMU.setSrd(9);
  }
  IMU.enableDataReadyInterrupt();

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(INTERRUPT_PIN, getIMU, RISING);

  IMU.enableFifo(true, true, true, false);
}

void initWebserver() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/zero", HTTP_GET, [](AsyncWebServerRequest *request){
    flash();
    offsetTime = micros();
    request->redirect("/");
//    request->send(200, "text/plain", "OK");
  });
  server.on("/madgwick", HTTP_GET, [](AsyncWebServerRequest *request){
    flash();
    offsetTime = micros();
    request->redirect("/");
//    request->send(200, "text/plain", "OK");
  });
  server.on("/mahony", HTTP_GET, [](AsyncWebServerRequest *request){
    flash();
    madgwick = false;
    request->redirect("/");
//    request->send(200, "text/plain", "OK");
  });
  server.on("/updaterate", HTTP_GET, [](AsyncWebServerRequest *request){
    flash();
    AsyncWebParameter* p = request->getParam(0);
    Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
    if(strcmp(p->name().c_str(), "hz") == 0) {
      updateDelay = 1000 / atoi(p->value().c_str());
    }
    request->redirect("/");
//    request->send(200, "text/plain", "OK");
  });
  server.on("/resetwifi", HTTP_GET, [](AsyncWebServerRequest *request){
    flash();
    WiFiManager wm;
    wm.resetSettings();
    request->send(200, "text/plain", "Rebooting...");
    delay(1000);
    ESP.restart();
    delay(1000);
  });

  server.onNotFound([](AsyncWebServerRequest *request){
    flash();
    Serial.print("NOT_FOUND ");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());
    request->send(404);
  });

  server.begin();
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Setup start...");

  sprintf(hostName, "esp-%04x%08x", (uint16_t)(chipid>>32), (uint32_t)chipid);

  Serial.printf("Set hostname %s.local\n", hostName);

  //set led pin as output
  pinMode(LED, OUTPUT);
  // start ticker with 0.6 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;
  //reset settings - for testing
  //wm.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wm.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wm.autoConnect(hostName)) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }

  ticker.detach();
  digitalWrite(LED, LOW);

  Serial.print("Got IP address ");
  Serial.println(WiFi.localIP());

  MDNS.begin(hostName);
  MDNS.addService("http", "tcp", 80);
  SPIFFS.begin();

  initIMU();
  initWebserver();

  Serial.println("Setup finished!");

  offsetTime = micros();
}

#define FIFO_BUFFER_SIZE 128
float ax[FIFO_BUFFER_SIZE];
float ay[FIFO_BUFFER_SIZE];
float az[FIFO_BUFFER_SIZE];

float gx[FIFO_BUFFER_SIZE];
float gy[FIFO_BUFFER_SIZE];
float gz[FIFO_BUFFER_SIZE];

float hx[FIFO_BUFFER_SIZE];
float hy[FIFO_BUFFER_SIZE];
float hz[FIFO_BUFFER_SIZE];

size_t fifo_samples[9];

size_t smallest(size_t *data, size_t remaining) {
  remaining--;
  if(remaining) {
    return min(data[remaining], smallest(data, remaining - 1));
  } else {
    return data[remaining];
  }
}

void loop() {
  if(newData) {
    IMU.readFifo();

    IMU.getFifoAccelX_mss(&fifo_samples[0], ax);
    IMU.getFifoAccelY_mss(&fifo_samples[1], ay);
    IMU.getFifoAccelZ_mss(&fifo_samples[2], az);

    IMU.getFifoGyroX_rads(&fifo_samples[3], gx);
    IMU.getFifoGyroY_rads(&fifo_samples[4], gy);
    IMU.getFifoGyroZ_rads(&fifo_samples[5], gz);

    IMU.getFifoMagX_uT(&fifo_samples[6], hx);
    IMU.getFifoMagX_uT(&fifo_samples[7], hy);
    IMU.getFifoMagX_uT(&fifo_samples[8], hz);
    newData = false;


    last = now;
    now = micros();

    char time[16];
    sprintf(time, "%011lu", now - offsetTime);

    size_t min_count = smallest(fifo_samples, 9);
    uint32_t interval = (now - last) / min_count;
    for(int c = 0; c < min_count; c++) {
//      Serial.printf("Accel  x %03.8f, y %03.8f, z %03.8f   --  ", ax[c], ay[c], az[c]);
//      Serial.printf("Gyro   x %03.8f, y %03.8f, z %03.8f   --  ", gx[c], gy[c], gz[c]);
//      Serial.printf("Magnet x %03.8f, y %03.8f, z %03.8f\n", hx[c], hy[c], hz[c]);

      if(madgwick == true) {
        MadgwickQuaternionUpdate(q, interval, ax[c], ay[c], az[c], gx[c], gy[c], gz[c], hx[c], hy[c], hz[c]);
      } else {
        MahonyQuaternionUpdate(q, interval, ax[c], ay[c], az[c], gx[c], gy[c], gz[c], hx[c], hy[c], hz[c]);
      }
    }
//    Serial.printf("%lu, %lu, %i, %lu, %i", now, last, interval, last - now, (last - now) / min_count);
    Serial.printf("%s (%05i, %02i): %03.8f, %03.8f, %03.8f, %03.8f\n", time, interval, min_count, q[0], q[1], q[2], q[3]);

    rootTx = jsonDoc.to<JsonObject>(); // start with an empty object
    rootTx["time"] = time;

//    rootTx["roll"] =
//    rootTx["pitch"] =
//    rootTx["yaw"] =

    rootTx["q0"] = q[0];
    rootTx["q1"] = q[1];
    rootTx["q2"] = q[2];
    rootTx["q3"] = q[3];

    String output = "";
    serializeJson(jsonDoc, output);
    ws.textAll(output);
  }

  delay(updateDelay);
  yield();

}
