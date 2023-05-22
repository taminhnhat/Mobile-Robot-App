#include <Arduino.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <math.h>

// define motor 1
#define MOTOR_1_A PB3
#define MOTOR_1_B PA15
#define MOTOR_1_PWM PB8
#define MOTOR_1_DIR PB9
// define motor 2
#define MOTOR_2_A PA12
#define MOTOR_2_B PA11
#define MOTOR_2_PWM PB4
#define MOTOR_2_DIR PB5
// define motor 3
#define MOTOR_3_A PC15
#define MOTOR_3_B PA0
#define MOTOR_3_PWM PB1
#define MOTOR_3_DIR PB0
// define motor 4
#define MOTOR_4_A PC13
#define MOTOR_4_B PC14
#define MOTOR_4_PWM PA5
#define MOTOR_4_DIR PA4

// define sensor
#define DIS_SEN PA1

// -------------------------------------------------JSON--------------------------------------------------------
StaticJsonDocument<200> doc;

// -------------------------------------------------SERIAL--------------------------------------------------------
HardwareSerial Serial2(PA3, PA2);

// -------------------------------------------------MOTOR--------------------------------------------------------
const float wheel_separation = 0.2; // m
const float wheel_distance = 0.146; // m
const float wheel_diameter = 0.09;  // m
const float motor_max_speed = 333;  // rpm
const float angular_velocity_factor = (wheel_distance * wheel_distance + wheel_separation * wheel_separation) / (2 * wheel_separation);
const float wheel_speed_factor = 60 / (3.1416 * wheel_diameter);
const uint32_t pwm_max_val = pow(2, PWM_RESOLUTION) - 1;
class MotorControl
{
private:
  const float ppr = 2970.0;
  uint32_t a;
  uint32_t b;
  uint32_t p;
  uint32_t dp;
  float v;
  uint32_t last_t;
  uint32_t last_p;
  uint32_t DIR_PIN_ADDR;
  uint32_t PWM_PIN_ADDR;
  double Setpoint, Input, Output;
  double aggKp = 4, aggKi = 0.2, aggKd = 1;
  double consKp = 1, consKi = 0.05, consKd = 0.25;
  struct pid_var
  {
    double Setpoint;
    double Input;
    double aggKp, aggKi, aggKd;
    double consKp, consKi, consKd;
  } pid;

  void tick()
  {
    const uint32_t d_t = millis() - this->last_t;
    if (d_t >= 100)
    {
      const uint32_t d_p = this->p - this->last_p;
      this->v = (d_p * 60) / (d_t * this->ppr);
      last_t = millis();
      last_p = this->p;
    }
  }

public:
  MotorControl(uint32_t direction_pin, uint32_t pwm_pin)
  {
    this->a = 0;
    this->b = 0;
    this->p = 0;
    this->last_t = millis();
    this->last_p = 0;
    this->DIR_PIN_ADDR = direction_pin;
    this->PWM_PIN_ADDR = pwm_pin;
  }
  MotorControl(uint32_t direction_pin, uint32_t pwm_pin, double aggKp, double aggKi, double aggKd, double consKp, double consKi, double consKd)
  {
    this->a = 0;
    this->b = 0;
    this->p = 0;
    this->last_t = millis();
    this->last_p = 0;
    this->DIR_PIN_ADDR = direction_pin;
    this->PWM_PIN_ADDR = pwm_pin;
    this->pid.aggKp = aggKp;
    this->pid.aggKi = aggKi;
    this->pid.aggKd = aggKd;
    this->pid.consKp = consKp;
    this->pid.consKi = consKi;
    this->pid.consKd = consKd;
  }
  void channel_a()
  {
    this->a++;
    this->p++;
    this->tick();
  }
  void channel_b()
  {
    this->b++;
    this->p++;
    this->tick();
  }
  void reset()
  {
    this->a = 0;
    this->b = 0;
  }
  uint32_t pulses()
  {
    return this->p;
  }
  float velocity()
  {
    return this->v;
  }
  void drive(float velocity)
  {
    const float wheel_speed = abs(velocity) * wheel_speed_factor;      // rpm
    const float pwm_val = wheel_speed * pwm_max_val / motor_max_speed; // %
    if (velocity > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, LOW);
      analogWrite(this->PWM_PIN_ADDR, pwm_val);
    }
    else
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, pwm_max_val - pwm_val);
    }
  }
  void fix()
  {
    //
  }
  void stop()
  {
    digitalWrite(this->DIR_PIN_ADDR, LOW);
    analogWrite(this->PWM_PIN_ADDR, LOW);
  }
} motor1(MOTOR_1_DIR, MOTOR_1_PWM),
    motor2(MOTOR_2_DIR, MOTOR_2_PWM),
    motor3(MOTOR_3_DIR, MOTOR_3_PWM),
    motor4(MOTOR_4_DIR, MOTOR_4_PWM);

void EncoderHandle_1_A()
{
  motor1.channel_a();
}
void EncoderHandle_1_B()
{
  motor1.channel_b();
}
void EncoderHandle_2_A()
{
  motor2.channel_a();
}
void EncoderHandle_2_B()
{
  motor2.channel_b();
}
void EncoderHandle_3_A()
{
  motor3.channel_a();
}
void EncoderHandle_3_B()
{
  motor3.channel_b();
}
void EncoderHandle_4_A()
{
  motor4.channel_a();
}
void EncoderHandle_4_B()
{
  motor4.channel_b();
}

// -------------------------------------------------PID CONTROLLER--------------------------------------------------------
// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;
PID servoCtrl(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
void angleGenerate(double Input)
{
  double gap = abs(Setpoint - Input); // distance away from setpoint
  if (gap < 10)
  { // we're close to setpoint, use conservative tuning parameters
    servoCtrl.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    // we're far from setpoint, use aggressive tuning parameters
    servoCtrl.SetTunings(aggKp, aggKi, aggKd);
  }

  servoCtrl.Compute();
}

// -------------------------------------------------PID CONTROLLER--------------------------------------------------------
uint32_t tick_t = millis();

String messageFromSerial1 = "";

void msgProcess(String);
void velocityProcess(float, float);

void setup()
{
  // Start serial 1 as external control communication
  Serial1.begin(115200);
  // Start serial 2 as rf communication
  Serial2.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIS_SEN, INPUT);
  pinMode(MOTOR_1_A, INPUT_PULLUP);
  pinMode(MOTOR_1_B, INPUT_PULLUP);
  pinMode(MOTOR_2_A, INPUT_PULLUP);
  pinMode(MOTOR_2_B, INPUT_PULLUP);
  pinMode(MOTOR_3_A, INPUT_PULLUP);
  pinMode(MOTOR_3_B, INPUT_PULLUP);
  pinMode(MOTOR_4_A, INPUT_PULLUP);
  pinMode(MOTOR_4_B, INPUT_PULLUP);
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  pinMode(MOTOR_3_PWM, OUTPUT);
  pinMode(MOTOR_4_PWM, OUTPUT);
  pinMode(MOTOR_1_DIR, OUTPUT);
  pinMode(MOTOR_2_DIR, OUTPUT);
  pinMode(MOTOR_3_DIR, OUTPUT);
  pinMode(MOTOR_4_DIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_A), EncoderHandle_1_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_B), EncoderHandle_1_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_A), EncoderHandle_2_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_B), EncoderHandle_2_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_A), EncoderHandle_3_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_B), EncoderHandle_3_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_A), EncoderHandle_4_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_B), EncoderHandle_4_B, HIGH);
}

void loop()
{
  if (millis() - tick_t > 10000)
  {
    Serial1.print(motor1.velocity());
    Serial1.print(':');
    Serial1.print(motor2.velocity());
    Serial1.print(':');
    Serial1.print(motor3.velocity());
    Serial1.print(':');
    Serial1.print(motor4.velocity());
    Serial1.println('.');
    tick_t = millis();
  }
}

void serialEvent1()
{
  while (Serial1.available())
  {
    char tempChar = (char)Serial1.read();
    if (tempChar != '\n')
    {
      messageFromSerial1 += tempChar;
    }
    else
    {
      msgProcess(messageFromSerial1);
      messageFromSerial1 = "";
    }
  }
}

void msgProcess(String lightCmd)
{
  const uint8_t len = lightCmd.length();
  char json[len];
  lightCmd.toCharArray(json, len);
  Serial1.println(json);
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  const char *topic = doc["topic"];
  const String topic_name = String(topic);
  if (topic_name.compareTo("control") == 0)
  {
    Serial1.println("control velocity");
    const double linear = doc["linear"];
    const double angular = doc["angular"];
    Serial1.print("linear: ");
    Serial1.print(linear);
    Serial1.print("\tangular: ");
    Serial1.println(angular);
  }
  else if (topic_name.compareTo("state") == 0)
  {
    //
  }
  else
  {
    //
  }
}

void velocityProcess(float linear, float angular)
{
  const float motor_1_velocity = linear - angular * angular_velocity_factor;
  const float motor_2_velocity = linear - angular * angular_velocity_factor;
  const float motor_3_velocity = linear + angular * angular_velocity_factor;
  const float motor_4_velocity = linear + angular * angular_velocity_factor;
  motor1.drive(motor_1_velocity);
  motor2.drive(motor_2_velocity);
  motor3.drive(motor_3_velocity);
  motor4.drive(motor_4_velocity);
}