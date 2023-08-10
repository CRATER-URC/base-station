#include <assert.h>

// Takes in one of three possible commands:
// 0  ------------> Disable all motors
// X Y  ----------> Controls PWM values on each side
// A B C D E F  --> Individually controls motor PWM values

constexpr int NUM_PINS = 6;
constexpr int PWM_PINS[]          = { 3,  4,  5,  6,  7,  8};
constexpr int ENABLE_PINS[]       = { 9, 10, 11, 12, 13, 14};
constexpr int DIRECTION_PINS[]    = {15, 16, 17, 18, 19, 20};
constexpr int DIRECTION_INVERTS[] = { 0,  0,  0,  0,  0,  0};

constexpr int TIMEOUT_MILLIS = 2000;
unsigned long last_command = 0;
bool stopped = true;

constexpr int MAX_PWM = 255;
constexpr int PORT = 9600;


void setup() {
  Serial.begin(PORT);
  assert(sizeof(PWM_PINS) > 0);
  assert(sizeof(PWM_PINS) / sizeof(PWM_PINS[0]) == NUM_PINS);
  assert(sizeof(PWM_PINS) == sizeof(ENABLE_PINS));
  assert(sizeof(PWM_PINS) == sizeof(DIRECTION_PINS));
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(ENABLE_PINS[i], OUTPUT);
    pinMode(DIRECTION_PINS[i], OUTPUT);
  }
}

void loop() {
  String read;
  if (Serial.available()) {
    last_command = millis();
    read = Serial.readString();
    int pwm_values[NUM_PINS];
    if (read.equals("0")) {
      stopped = true;
    } else if (read.indexOf(' ') == read.lastIndexOf(' ') && read.indexOf(' ') > 0) {
      int space = read.indexOf(' ');
      String str1 = read.substring(0, space);
      String str2 = read.substring(space + 1);
      int val1 = str1.toInt();
      int val2 = str1.toInt();
      for (int i = 0; i < NUM_PINS / 2; i++) {
        pwm_values[i] = val1;
        pwm_values[NUM_PINS - 1 - i] = val2;
      }
    } else {
      String strs[NUM_PINS];
      String subread = read;
      for (int i = 0; i < NUM_PINS - 1; i++) {
        int space = subread.indexOf(' ');
        if (space <= 0) {
          stopped = true;
          break;
        }
        strs[i] = subread.substring(0, space);
        subread = subread.substring(space + 1);
      }
      if (subread.indexOf(' ') > 0) {
        stopped = true;
      } else {
        strs[NUM_PINS - 1] = subread;
      }
      if (!stopped) {
        for (int i = 0; i < NUM_PINS; i++) {
          pwm_values[i] = strs[i].toInt();
        }
      }
    }
    Serial.println(read);
    for (int i = 0; i < NUM_PINS; i++) {
      if (stopped) {
        digitalWrite(ENABLE_PINS[i], LOW);
        pwm_values[i] = 0;
      } else {
        digitalWrite(ENABLE_PINS[i], HIGH);
      }
      analogWrite(PWM_PINS[i], constrain(abs(pwm_values[i]), 0, MAX_PWM));
      if ((DIRECTION_INVERTS[i] == 1) == (pwm_values[i] < 0)) {
        digitalWrite(DIRECTION_PINS[i], LOW);
      } else {
        digitalWrite(DIRECTION_PINS[i], HIGH);
      }
    }
    Serial.println("\nINPUT:  " + read);
    Serial.println("OUTPUT: " + pwm_values[0]);
    for (int i = 1; i < NUM_PINS; i++) {
      Serial.print(" " + pwm_values[i]);
    }
  } else if ((millis() - last_command) >= TIMEOUT_MILLIS && !stopped) {
    for (int i = 0; i < NUM_PINS; i++) {
      analogWrite(PWM_PINS[i], 0);
      digitalWrite(ENABLE_PINS[i], LOW);
    }
    Serial.println("MOTOR TIMEOUT");
  }
}