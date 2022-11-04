#pragma once
#ifndef myTRUMA_H
#define myTRUMA_H
#include <Arduino.h>

#define trumaMsgLen 16

struct truma {
  uint8_t code;
  char text[16];
};

const PROGMEM truma ENERGY_MIX_MAPPING[] = {     //ID:20  Data:3
  {0x00, "electricity"},
  {0xFA, "gas/mix"}};

const PROGMEM truma ENERGY_MODE_MAPPING[] = {    //ID:20  Data:4
  {0x00, "gas"},
  {0x09, "mix/electric 1"},
  {0x12, "mix/electric 2"}};

const PROGMEM truma ENERGY_MODE_2_MAPPING[] = {  //ID:20  Data:5&0F
  {0x01, "Gas"},
  {0x02, "Electric"},
  {0x03, "Gas/Electric"}};

const PROGMEM truma VENT_MODE_MAPPING[] = {      //ID:20  Data:5 >> 4
  {0x00, "Off"},
  {0x0B, "Eco"},
  {0x0D, "High"},
  {0x01, "Vent 1"}, {0x02, "Vent 2"},
  {0x03, "Vent 3"}, {0x04, "Vent 4"},
  {0x05, "Vent 5"}, {0x06, "Vent 6"},
  {0x07, "Vent 7"}, {0x08, "Vent 8"},
  {0x09, "Vent 9"}, {0x0A, "Vent 10"}};

const PROGMEM truma VENT_OR_OPERATING_STATUS[] = {  //ID:21(61)  Data:5
  {0x01, "off"},
  {0x22, "on + airvent"},
  {0x02, "on"},
  {0x31, "error (?)"},
  {0x32, "fatal error"},
  {0x21, "airvent (?)"}};

const PROGMEM truma CP_PLUS_DISPLAY_STATUS_MAPPING[] = {  //ID:22(e2)  Data:1
  {0xF0, "heating on"},
  {0x20, "standby ac on"},
  {0x00, "standby ac off"},
  {0xD0, "error"},
  {0x70, "fatal error"},
  {0x50, "boiler on"},
  {0x40, "boiler off"}};

const PROGMEM truma HEATING_STATUS_MAPPING[] = {    //ID:22(e2)  Data:2
  {0x10, "boiler eco done"},
  {0x11, "boiler eco heat"},
  {0x30, "boiler hot done"},
  {0x31, "boiler hot heat"}};

const PROGMEM truma HEATING_STATUS_2_MAPPING[] = {  //ID:22(e2)  Data:3
  {0x04, "normal"},
  {0x05, "error"},
  {0xFF, "fatal error (?)"},
  {0xFE, "normal (?)"}};

const char truma_unbekannt[] = "????";

char trumaTmpMsg[trumaMsgLen];
uint8_t trumaMapCode;

char* trumaMap(const truma map[], size_t size, uint8_t code) {
  for (size_t i = 0; i < size; i++) {
    memcpy_P(&trumaMapCode, &map[i].code,1);
    if (trumaMapCode == code) {
      strcpy_P(trumaTmpMsg,(char *)map[i].text);
      //DEBUG__PRINTF("trumaMap - %02x %s\n", code,trumaTmpMsg);
      return trumaTmpMsg;
    }
  }
  return (char *)truma_unbekannt;
}

#endif