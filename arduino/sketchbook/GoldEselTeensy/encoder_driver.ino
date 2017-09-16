#include "encoder_driver.h"

volatile long enc_pos = 0L;

static const int8_t ENC_STATES [] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; //encoder lookup table

// Interrupt routine for encoder, taking care of actual counting
void isr() {
  static uint8_t enc_last = 0;
  uint8_t b = digitalRead(ENC_PIN_B);
  uint8_t a = digitalRead(ENC_PIN_A);

  enc_last <<= 2;
  enc_last |= a << 1 | b;

  enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

long readEncoder() {
  long ret = enc_pos;
  enc_pos = 0L;

  return ret;
}

void initEncoder() {
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), isr, CHANGE);
}

