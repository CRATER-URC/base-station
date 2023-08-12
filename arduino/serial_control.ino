#include <assert.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

// Takes in one of three possible commands:
// 0  ------------> Disable all motors
// X Y  ----------> Controls PWM values on each side
// A B C D E F  --> Individually controls motor PWM values

constexpr int NUM_PINS = 6;
constexpr int PWM_PINS[]          = { 3,  4,  5,  6,  7,  8};
constexpr int ENABLE_PINS[]       = { 9, 10, 11, 12, 13, 14};
constexpr int DIRECTION_PINS[]    = {15, 16, 17, 18, 19, 20};
constexpr int DIRECTION_INVERTS[] = { 0,  0,  0,  0,  0,  0};

constexpr int TIMEOUT_MILLIS = 4000;
unsigned long last_command = 0;
bool stopped = true;

constexpr int MAX_PWM = 255;
constexpr int PORT = 9600;

float left_vel = 0.0;
float right_vel = 0.0;

void vel_L_cb( const std_msgs::Float32& cmd_msg){
  left_vel = cmd_msg.data; //float  
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  last_command = millis();
  }

void vel_R_cb( const std_msgs::Float32& cmd_msg){
  right_vel = cmd_msg.data; //float  
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  last_command = millis();
  }

  ros::Subscriber<std_msgs::Float32> sub_L("vel_L", vel_L_cb);
  ros::Subscriber<std_msgs::Float32> sub_R("vel_R", vel_R_cb);

  // std_msgs::Float64 myData;
  // ros::Publisher sendData("float_data", &myData);

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

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub_L);
  nh.subscribe(sub_R);
}

void loop() {
  String read;
  if (Serial.available()) {
    last_command = millis();
    stopped = false;
    read = Serial.readString();
    int pwm_values[NUM_PINS];

    if (read.equals("0")) {
      stopped = true;
    } else if (read.indexOf(' ') == read.lastIndexOf(' ') && read.indexOf(' ') > 0) {
      int space = read.indexOf(' ');
      String str1 = read.substring(0, space);
      String str2 = read.substring(space + 1);
      int val1 = str1.toInt();
      int val2 = str2.toInt();
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
    for (int i = 0; i < NUM_PINS; i++) {
      if (stopped) {
        digitalWrite(ENABLE_PINS[i], LOW);
        pwm_values[i] = 0;
      } else {
        pwm_values[i] = constrain(pwm_values[i], -MAX_PWM, MAX_PWM);
        digitalWrite(ENABLE_PINS[i], HIGH);
      }
      analogWrite(PWM_PINS[i], abs(pwm_values[i]));
      if ((DIRECTION_INVERTS[i] == 1) == (pwm_values[i] < 0)) {
        digitalWrite(DIRECTION_PINS[i], LOW);
      } else {
        digitalWrite(DIRECTION_PINS[i], HIGH);
      }
    }
    Serial.println("INPUT:  " + read);
    Serial.print("OUTPUT: " + (String) pwm_values[0]);
    for (int i = 1; i < NUM_PINS; i++) {
      Serial.print(" " + (String) pwm_values[i]);
    }
    Serial.println("\n");
  } else if ((millis() - last_command) >= TIMEOUT_MILLIS && !stopped) {
    for (int i = 0; i < NUM_PINS; i++) {
      analogWrite(PWM_PINS[i], 0);
      digitalWrite(ENABLE_PINS[i], LOW);
    }
    Serial.println("MOTOR TIMEOUT");
  }
  nh.spinOnce();
  delay(1);
}