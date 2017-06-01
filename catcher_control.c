#include "catcher_control.h"

void lower_catcher() {
  TCCR3A |= (1<<COM3A1);
  OCR3A = 375;
}

void raise_catcher() {
  TCCR3A |= (1<<COM3A1);
  OCR3A = 250;
}

void catcher_limit_switch() {
  TCCR3A &= ~(1<<COM3A1);
}
