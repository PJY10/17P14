#include <Servo.h>

// Arduino pin assignment

#define PIN_IR    0         // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 600   // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500  // servo neutral position (90 degree)
#define _DUTY_MAX 2400  // servo full counter-clockwise position (180 degree)

#define _DIST_MIN  100.0   // minimum distance 100mm
#define _DIST_MAX  250.0   // maximum distance 250mm

#define EMA_ALPHA  0.20    // for EMA Filter

#define INTERVAL 20

Servo myservo;

unsigned long last_time = 0;

float dist_raw = 0.0;
float dist_ema = 0.0;

void setup() {
  Serial.begin(1000000);
  pinMode(PIN_LED, OUTPUT);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  dist_ema = (_DIST_MIN + _DIST_MAX) * 0.5;
}

void loop() {
  unsigned long now = millis();
  if (now - last_time < INTERVAL) return;
  last_time = now;

  int a_value = analogRead(PIN_IR);

  dist_raw = (6762.0 / ((float)a_value - 9.0) - 4.0) * 10.0 - 60.0;

  if (dist_raw < 0) dist_raw = 0;

  dist_ema = EMA_ALPHA * dist_raw + (1.0 - EMA_ALPHA) * dist_ema;

  if (dist_ema >= _DIST_MIN && dist_ema <= _DIST_MAX) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  float d = dist_ema;
  if (d < _DIST_MIN) d = _DIST_MIN;
  if (d > _DIST_MAX) d = _DIST_MAX;

  float duty_f =
    _DUTY_MIN
    + (d - _DIST_MIN)
      * (float)(_DUTY_MAX - _DUTY_MIN)
      / (_DIST_MAX - _DIST_MIN);

  int duty = (int)(duty_f + 0.5);

  myservo.writeMicroseconds(duty);

  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}
