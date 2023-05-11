#include <Arduino.h>

// define motor 1
#define MOTOR_1_A 13
#define MOTOR_1_B 12
#define MOTOR_1_PWM 38
// define motor 2
#define MOTOR_2_A 11
#define MOTOR_2_B 10
#define MOTOR_2_PWM 39
// define motor 3
#define MOTOR_3_A 7
#define MOTOR_3_B 4
#define MOTOR_3_PWM 40
// define motor 4
#define MOTOR_4_A 3
#define MOTOR_4_B 2
#define MOTOR_4_PWM 41

// define sensor
#define ENDSTOP 19
#define AMP_SENSOR 18

int drive(int);
int stop();

void setup()
{
  // Start serial 1 as external control communication
  Serial.begin(115200);
  // Start serial 2 as rf communication
  Serial1.begin(115200);
  pinMode(MOTOR_1_A, INPUT);
  pinMode(MOTOR_1_B, INPUT);
  pinMode(MOTOR_2_A, INPUT);
  pinMode(MOTOR_2_B, INPUT);
  pinMode(MOTOR_3_A, INPUT);
  pinMode(MOTOR_3_B, INPUT);
  pinMode(MOTOR_4_A, INPUT);
  pinMode(MOTOR_4_B, INPUT);

  // attachInterrupt(digitalPinToInterrupt())

  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  pinMode(MOTOR_3_PWM, OUTPUT);
  pinMode(MOTOR_4_PWM, OUTPUT);
}

void loop()
{
  Serial.print('.');
  Serial1.print('.');
  delay(1000);
}

int drive(int speed)
{
  return 1;
}

int stop()
{
  return 1;
}