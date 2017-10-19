//  Goldesel (German for gold-donkey from the Grimm Brothers fairy tale "Table-Be-Set, Gold-Donkey, and Cudgel-out-of-the-Sack")
// http://www.pitt.edu/~dash/grimm036.html
//
#include <AStar32U4.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ENC_PIN_A 8  // pin D8
#define ENC_PIN_B 9  // pin D9

volatile long enc_pos = 0L;

long enc_sum = 0L;
static const int8_t ENC_STATES [] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; //encoder lookup table
unsigned long enc_last_time = 0;

// Interrupt routine for encoder, taking care of actual counting
ISR(PCINT0_vect) {
  static uint8_t enc_last = 0;
  bool b = FastGPIO::Pin<ENC_PIN_B>::isInputHigh();
  bool a = FastGPIO::Pin<ENC_PIN_A>::isInputHigh();

  enc_last <<= 2;
  enc_last |= ((uint8_t)a) << 1 | (uint8_t)b;

  enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

static void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR |= 1; // clear any outstanding interrupt
    *digitalPinToPCICR(pin) |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

long readEncoder() {
  long ret = enc_pos;
  enc_pos = 0L;

  return ret;
}

void initEncoder() {
  FastGPIO::Pin<ENC_PIN_A>::setInputPulledUp();
  FastGPIO::Pin<ENC_PIN_B>::setInputPulledUp();
  pciSetup(ENC_PIN_A);
  pciSetup(ENC_PIN_B);
//  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), encISR, CHANGE);
}

#define ON HIGH
#define OFF LOW

#define TURN_L 4
#define TURN_R 7
#define HEAD   5
#define BRKE   6

unsigned long last_command = 0;
unsigned long blinking_time = 0;
unsigned char blinking_state = 0;
boolean auto_blinking = false;
boolean blinking = false;
int blinking_on = OFF;
// Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);

//   if (!bno.begin()) {
//     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//     while(1);
//   }

  last_command = millis();

  pinMode(TURN_L, OUTPUT);
  pinMode(TURN_R, OUTPUT);
  pinMode(HEAD, OUTPUT);
  pinMode(BRKE, OUTPUT);

  digitalWrite(TURN_L, OFF);
  digitalWrite(TURN_R, OFF);
  digitalWrite(HEAD, OFF);
  digitalWrite(BRKE, OFF);

  initEncoder();
}

void loop() {
  if (auto_blinking && Serial.available() > 0) {
    auto_blinking = false;

    digitalWrite(TURN_L, OFF);
    digitalWrite(TURN_R, OFF);
    digitalWrite(HEAD, OFF);
    digitalWrite(BRKE, OFF);
    // Serial.println("End Blinking");
  }

  while (Serial.available() > 0) {
    int incomingByte = Serial.read();

    last_command = millis();

    switch (incomingByte) {
      case 'B':
        digitalWrite(BRKE, ON);
        break;
      case 'b':
        digitalWrite(BRKE, OFF);
        break;
      case 'E':
        digitalWrite(TURN_L, ON);
        digitalWrite(TURN_R, ON);
        blinking_time = millis();
        blinking_state = 3;
        blinking_on = ON;
        blinking = true;
        break;
      case 'e':
        digitalWrite(TURN_L, OFF);
        digitalWrite(TURN_R, OFF);
        blinking_state = 0;
        blinking_on = OFF;
        blinking = false;
        break;
      case 'H':
        digitalWrite(HEAD, ON);
        break;
      case 'h':
        digitalWrite(HEAD, OFF);
        break;
      case 'L':
        if ((blinking_state & 1) == 0) {
          digitalWrite(TURN_L, ON);
          blinking_time = millis();
          blinking_state |= 1;
          blinking_on = ON;
          blinking = true;
        }
        break;
      case 'l':
        digitalWrite(TURN_L, OFF);
        blinking_state &= ~1;
        if (blinking_state == 0) {
          blinking_on = OFF;
          blinking = false;
        }
        break;
      case 'R':
        if ((blinking_state & 2) == 0) {
          digitalWrite(TURN_R, ON);
          blinking_time = millis();
          blinking_state = 2;
          blinking_on = ON;
          blinking = true;
        }
        break;
      case 'r':
        digitalWrite(TURN_R, OFF);
        blinking_state &= ~2;
        if (blinking_state == 0) {
          blinking_on = OFF;
          blinking = false;
        }
        break;
      case 'S':
      case 's':
        digitalWrite(TURN_L, OFF);
        digitalWrite(TURN_R, OFF);
        digitalWrite(HEAD, OFF);
        digitalWrite(BRKE, OFF);
        blinking_state = 0;
        blinking_on = OFF;
        blinking = false;
        break;
    }
  }

  if (blinking && millis() - blinking_time > 200) {
    blinking_time = millis();

    if (blinking_on == ON) {
      blinking_on = OFF;
    }
    else {
      blinking_on = ON;
    }

    if (blinking_state & 1) {
      digitalWrite(TURN_L, blinking_on);
    }
    if (blinking_state & 2) {
      digitalWrite(TURN_R, blinking_on);
    }
  }

  if (!auto_blinking && millis() - last_command >  60000) {
    auto_blinking = true;
    blinking = false;
    blinking_time = millis();

    digitalWrite(TURN_L, OFF);
    digitalWrite(TURN_R, OFF);
    digitalWrite(HEAD, OFF);
    digitalWrite(BRKE, OFF);
    // Serial.println("Start Blinking");
  }

  if (auto_blinking && millis() - blinking_time > 200) {
    blinking_time = millis();
    if (blinking_state == 0) {
      blinking_state = 1;
    }
    else {
      blinking_state <<= 1;
    }

    // Serial.print("Blink: "); Serial.println(blinking_state);
    switch (blinking_state) {
      case 0x01:
        digitalWrite(TURN_L, ON);
        break;
      case 0x02:
        digitalWrite(TURN_L, OFF);
        break;
      case 0x04:
        digitalWrite(TURN_R, ON);
        break;
      case 0x08:
        digitalWrite(TURN_R, OFF);
        break;
      case 0x10:
        digitalWrite(HEAD, ON);
        break;
      case 0x20:
        digitalWrite(HEAD, OFF);
      case 0x40:
        digitalWrite(BRKE, ON);
        break;
      case 0x80:
        digitalWrite(BRKE, OFF);
        break;
    }
  }

  long time_stamp = millis();
  long dt = time_stamp - enc_last_time;

  if (dt >= 100) {
    long e = readEncoder();

    enc_sum += e;

    Serial.print("E "); Serial.print(e); Serial.print(" "); Serial.print(dt); Serial.print(" ");
    Serial.println(enc_sum);

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
//     imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

//     Serial.print("L ");
    // Reordered! X Y Z
//     Serial.print(- accel.y(), 3); Serial.print(" "); Serial.print(accel.x(), 3); Serial.print(" "); Serial.print(accel.z(), 3);
//     Serial.print(" "); Serial.println(dt);

    enc_last_time = time_stamp;
  }

  delay(1);
}
