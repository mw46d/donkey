#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <PulsePosition.h>

#define LED_GREEN 0
#define LED_YELLOW 1
#define RPM_SENSOR 2
#define LED_RED 3

#define RC_IN 5

#define EXTRA_PIN 9

#define RC_OUT1 22
#define RC_OUT3 23

#define THROTTLE_NEUTRAL 1500
#define THROTTLE_BRAKE   1385
#define STEERING_VALUE   1446

#ifdef STRAIGHT_STEERING
#define STEER(x) (x)
#else
#define STEER(x) (((x) == 0.0) ? 0.0 : 3000.0 - (x))
#endif

PulsePositionOutput servoThrottle;
PulsePositionOutput servoSteering;
PulsePositionInput rcInput;

RF24 radio(7,8);
byte addresses[][6] = {"Entd1", "Entd2"};

#define COMMAND_LEN 8

char command_start[COMMAND_LEN] = { 'G', 'O', 0, 0, 0, 0, 0, 0 };
char command_stop[COMMAND_LEN] =  { 'S', 'T', 'O', 'P', 0, 0, 0, 0 };
char command_ok[COMMAND_LEN] =  { 'O', 'K', 0, 0, 0, 0, 0, 0 };

char rx_buff = 0;
unsigned long rx_time = 0;
unsigned long yellow_time = 0;
unsigned long go_time = 0;
unsigned long stop_time = 0;
bool going = false;
bool stopped = true;
unsigned long last_rpm_duration = -1;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello Teensy");

  rcInput.begin(RC_IN);
  servoThrottle.begin(RC_OUT1);
  servoSteering.begin(RC_OUT3);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);

  pinMode(RPM_SENSOR, INPUT_PULLUP);
  pinMode(EXTRA_PIN, INPUT_PULLUP);

  servoSteering.write(STEERING_VALUE);
  servoThrottle.write(THROTTLE_NEUTRAL);
  delay(3000);
  servoThrottle.write(THROTTLE_BRAKE);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(0, 15);
  radio.setPayloadSize(COMMAND_LEN);
  radio.setAutoAck(true);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}

void loop() {
  bool notBypass = digitalRead(EXTRA_PIN);
  unsigned long time = millis();

  if ((rx_time < time - 3000 || rx_buff != 'G') && notBypass) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);

    if (going) {
      going = false;
      stopped = false;
      last_rpm_duration = -1;
      stop_time = time;
    }

    if (servoSteering.read() != STEERING_VALUE) {
      servoSteering.write(STEERING_VALUE);
    }
    
    unsigned long dtime = stop_time - time;

    if (dtime < 200) {
      if (servoThrottle.read() != THROTTLE_NEUTRAL) {
        servoThrottle.write(THROTTLE_NEUTRAL);
      }
    }
    else {
      if (!stopped) {
        unsigned long duration = pulseIn(RPM_SENSOR, HIGH, 160000);

        if (duration > 0 && duration < 100000 &&
            (last_rpm_duration == -1 || last_rpm_duration <= duration + 50)) {
          last_rpm_duration = duration;
          digitalWrite(LED_GREEN, HIGH);

          if (servoThrottle.read() != THROTTLE_BRAKE) {
            servoThrottle.write(THROTTLE_BRAKE);
          }
        }
        else {
          stopped = true;
        }
      }

      if (stopped && servoThrottle.read() != THROTTLE_NEUTRAL) {
        servoThrottle.write(THROTTLE_NEUTRAL);
      }
    }
  }
  else {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);

    static float inSteering = 0.0;
    static float inThrottle = 0.0;
    static unsigned long rcTime = 0;
    
    byte count = rcInput.available();

    if (count > 0) {
      inSteering = STEER(rcInput.read(1));
      inThrottle = rcInput.read(3);
      rcTime = time;
    }

    if (!going) {
      servoThrottle.write(THROTTLE_NEUTRAL);
      go_time = time;
      going = true;
    }

    static unsigned int piSteering = 0;
    static unsigned int piThrottle = 0;
    static unsigned long piTime = 0;

    while (Serial.available() >= 10) {
      char b = Serial.read();

      if (b == 'S') {
        // Steering value in us
        // S <steer>\n
        piSteering = Serial.parseInt();
        piTime = time;
      }
      else if (b == 'T') {
        // Throttle value in us
        // T <throttle>\n
        piThrottle = Serial.parseInt();
        piTime = time;
      }

      Serial.read();
    }

    unsigned long dtime = go_time - time;

    if (dtime > 200) {
      Serial.print("I "); Serial.print(inSteering, 1); Serial.print("  "); Serial.print(inThrottle, 1); Serial.print("  ");
      Serial.print(rcInput.read(5), 1); Serial.print("  "); Serial.print(rcInput.read(6), 1); Serial.print("  ");
      Serial.print(rcInput.read(7), 1); Serial.print("  "); Serial.println(rcInput.read(8), 1);

      if (time - rcTime > 100) {
        inSteering = STEERING_VALUE;
        inThrottle = THROTTLE_NEUTRAL;
      }
      
      if (time - piTime < 100) {
        if (piSteering != 0 && piThrottle != 0) {
          inSteering = piSteering;
          inThrottle = piThrottle;
        }
      }
      else {
        piSteering = 0;
        piThrottle = 0;
      }

      servoThrottle.write(inThrottle);
      servoSteering.write(inSteering);
    }
  }

  if (rx_time > time - 500 ) {
    digitalWrite(LED_YELLOW, LOW);
    yellow_time = 0;
  }
  else {
    if (yellow_time == 0) {
      yellow_time = time;
    }

    if ((time - yellow_time) % 200 < 100) {
      digitalWrite(LED_YELLOW, HIGH);
    }
    else {
      digitalWrite(LED_YELLOW, LOW);
    }
  }

  if (radio.available()) {
    char received[COMMAND_LEN];
    bool got_packet = false;

    while (radio.available()) {
      if (radio.getDynamicPayloadSize() < 1) {
        // Corrupt payload has been flushed
      }
      else {
        radio.read(received, COMMAND_LEN);
        got_packet = true;
      }
    }

    if (got_packet) {
      rx_time = millis();
      rx_buff = received[0];
    }

    radio.stopListening();
    radio.write(command_ok, COMMAND_LEN);
    radio.startListening();
  }

  unsigned long time_stop = millis();

  if (time_stop - time < 10) {
    delay(10 + time - time_stop);
  }
}
