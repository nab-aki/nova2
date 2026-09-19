#include "hal_light.h"

#include "hal_pins.h"

void Light_Setup(void) {
  pinMode(PIN_LIGHT, INPUT);
}

int Light_Read(void) {
  return analogRead(PIN_LIGHT);
}
