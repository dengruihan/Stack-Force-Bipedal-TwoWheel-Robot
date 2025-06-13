#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "bipedal_data.h"
#include "CAN_comm.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ppm.h"
#include "Motor.h"
#include "robot.h"
#include "can.h"
#include "pid.h"

#define USE_WEB_SERVER
#ifdef USE_WEB_SERVER
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include "basic_web.h"
#include "wifi_config.h"

void web_loop();
void basicWebCallback(void);
void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
#endif