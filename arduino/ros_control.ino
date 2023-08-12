#include <assert.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

constexpr int NUM_PINS = 6;
constexpr int PWM_PINS[]          = { 3,  4,  5,  6,  7,  8};
constexpr int ENABLE_PINS[]       = { 9, 10, 11, 12, 13, 14};
constexpr int DIRECTION_PINS[]    = {15, 16, 17, 18, 19, 20};
constexpr int DIRECTION_INVERTS[] = { 0,  0,  0,  0,  0,  0};

constexpr int TIMEOUT_MILLIS = 200;
unsigned long last_command = 0;

constexpr int MAX_PWM = 255;
constexpr int PORT = 9600;

float left_vel = 0.0;
float right_vel = 0.0;

ros::NodeHandle nh;

void vel_L_cb( const std_msgs::Float32& cmd_msg) {
  left_vel = cmd_msg.data;  // float
  last_command = millis();

  for (int i = 0; i < NUM_PINS / 2; i++) {
    digitalWrite(ENABLE_PINS[i], HIGH);
    int left_pwm = (MAX_PWM * left_vel);
    analogWrite(PWM_PINS[i], abs(left_pwm));
    if ((DIRECTION_INVERTS[i] == 1) == (left_pwm < 0)) {
      digitalWrite(DIRECTION_PINS[i], LOW);
    } else {
      digitalWrite(DIRECTION_PINS[i], HIGH);
    }
  }
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void vel_R_cb( const std_msgs::Float32& cmd_msg) {
  right_vel = cmd_msg.data;  // float
  last_command = millis();
  
  for (int i = NUM_PINS / 2; i < NUM_PINS; i++) {
    digitalWrite(ENABLE_PINS[i], HIGH);
    int right_pwm = (MAX_PWM * right_vel);
    analogWrite(PWM_PINS[i], abs(right_pwm));
    if ((DIRECTION_INVERTS[i] == 1) == (right_pwm < 0)) {
      digitalWrite(DIRECTION_PINS[i], LOW);
    } else {
      digitalWrite(DIRECTION_PINS[i], HIGH);
    }
  }
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}

ros::Subscriber<std_msgs::Float32> sub_L("vel_L", vel_L_cb);
ros::Subscriber<std_msgs::Float32> sub_R("vel_R", vel_R_cb);

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

  // pinMode(13, OUTPUT);  // LED
  nh.initNode();
  nh.subscribe(sub_L);
  nh.subscribe(sub_R);
}


void loop() {  
  if ((millis() - last_command) >= TIMEOUT_MILLIS) {
    for (int i = 0; i < NUM_PINS; i++) {
      analogWrite(PWM_PINS[i], 0);
      digitalWrite(ENABLE_PINS[i], LOW);
    }
  }
  nh.spinOnce();
  delay(1);
}