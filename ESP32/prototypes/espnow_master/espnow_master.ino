#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

//#include "espnow_handle.h"

typedef struct message{
  char message[100];
} message;

message incoming_message;
message outgonig_message;

uint8_t broadcastAddress[] = {0x7C, 0x87, 0xCE, 0x27, 0xEF, 0x44};

#define N_SLAVES 1
bool new_command = false;

esp_now_peer_info_t slaves[N_SLAVES];

bool waiting = false;
unsigned long start_millis;
void OnMessageSent(const uint8_t *mac_address, esp_now_send_status_t status){
  start_millis = millis();
  if (status == 0){
    waiting = true;
  }
}

void OnMessageRecv(const uint8_t *mac_address, const uint8_t *incomingMessage, int len){
  waiting = false;
  waiting = false;
  memcpy(&incoming_message, incomingMessage, len);
  //String inputString = parse_serial(incoming_message.message, &new_command);
  //parse_string(inputString);
  Serial.print(millis() - start_millis);
}

void parse_string(String input_string){
  Serial.println(input_string);
}
unsigned long begin_millis;
void setup() {
  Serial.begin(230400);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnMessageSent);
  esp_now_register_recv_cb(OnMessageRecv);
  memcpy(slaves[0].peer_addr, broadcastAddress, 6);
  slaves[0].channel = 0;
  slaves[0].encrypt = false;
  esp_now_add_peer(&slaves[0]);
  begin_millis = millis();
}

void loop() {
  
  if (millis() - begin_millis >= 1000){
    String command = "message sent!";
    strcpy(outgonig_message.message, command.c_str());
    esp_now_send(broadcastAddress, (uint8_t *) &outgonig_message, sizeof(outgonig_message));
    begin_millis = millis();
  }
}
