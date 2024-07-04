#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "espnow_handle.h"

typedef struct message{
  char message[100];
} message;

message incoming_message;
message outgonig_message;

uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x38, 0x28, 0xF8};

#define N_SLAVES 1
bool new_command = false;

esp_now_peer_info_t slaves[N_SLAVES];

bool waiting = false;
void OnMessageSent(const uint8_t *mac_address, esp_now_send_status_t status){
  Serial.println("sending");
  if (status == 0){
    waiting = true;
    Serial.println("sent");
  }
}

void OnMessageRecv(const uint8_t *mac_address, const uint8_t *incomingMessage, int len){
  waiting = false;
  memcpy(&incoming_message, incomingMessage, len);
  String inputString = parse_serial(incoming_message.message, &new_command);
  parse_string(inputString);
 
  String command = "message sent!";
  strcpy(outgonig_message.message, command.c_str());
  esp_now_send(broadcastAddress, (uint8_t *) &outgonig_message, sizeof(outgonig_message));
}

void parse_string(String input_string){
  Serial.println(input_string);
}

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
}

void loop() {

}
