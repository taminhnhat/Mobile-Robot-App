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
#define MOTOR_3_A PA6
#define MOTOR_3_B PA7
#define MOTOR_3_PWM PB1
#define MOTOR_3_DIR PB0
// define motor 4
#define MOTOR_4_A PC13
#define MOTOR_4_B PA0
#define MOTOR_4_PWM PA5
#define MOTOR_4_DIR PA4

// define sensor
#define DIS_SEN PA1

#define FRONT_LIGHT PC13

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
const float max_linear_velocity = motor_max_speed / wheel_speed_factor;
const float max_angular_velocity = (2 * motor_max_speed) / (wheel_speed_factor * wheel_separation);
const uint32_t pwm_max_val = pow(2, PWM_RESOLUTION) - 1;
class MotorControl
{
private:
  const float ppr = 330.0;
  const uint32_t vel_cal_cycle = 50;
  uint32_t a;
  uint32_t b;
  uint32_t p;
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

public:
  uint32_t d_p;

  void tick(uint32_t d_t)
  {
    const uint32_t present_p = this->p;
    this->d_p = present_p - this->last_p;
    this->v = (this->d_p * 60) / (d_t * this->ppr);
    this->last_p = present_p;
  }
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
  }
  void channel_b()
  {
    this->b++;
    this->p++;
  }
  void reset()
  {
    this->a = 0;
    this->b = 0;
    this->p = 0;
  }
  uint32_t pulses()
  {
    return this->p;
  }
  float velocity()
  {
    return this->v;
  }
  float speed()
  {
    return this->v * wheel_speed_factor;
  }
  void drive(float velocity)
  {
    const float wheel_speed = abs(velocity) * wheel_speed_factor;         // rpm
    const uint32_t pwm_val = wheel_speed * pwm_max_val / motor_max_speed; // %
    if (velocity > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, pwm_max_val - pwm_val);
    }
    else
    {
      digitalWrite(this->DIR_PIN_ADDR, LOW);
      analogWrite(this->PWM_PIN_ADDR, pwm_val);
    }
  }
  void rotate(float speed)
  {
    float wheel_speed = abs(speed); // rpm
    if (wheel_speed > motor_max_speed)
      wheel_speed = motor_max_speed;
    uint32_t pwm_val = wheel_speed * pwm_max_val / motor_max_speed; // %
    if (speed > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, pwm_max_val - pwm_val);
    }
    else
    {
      digitalWrite(this->DIR_PIN_ADDR, LOW);
      analogWrite(this->PWM_PIN_ADDR, pwm_val);
    }
  }
  void write(uint32_t duty, uint32_t level)
  {
    digitalWrite(this->DIR_PIN_ADDR, level);
    analogWrite(this->PWM_PIN_ADDR, duty);
  }
  void lock()
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
void velocityProcess(double, double);

// -------------------------------------------------TIMER--------------------------------------------------------
HardwareTimer timer(TIM1);
uint64_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 50 == 0)
  {
    motor1.tick(50);
    motor2.tick(50);
    motor3.tick(50);
    motor4.tick(50);
  }
  else if (timer_count % 1000 == 0)
  {
    Serial1.println("timer tick!");
  }
}

// -------------------------------------------------MAIN CODE--------------------------------------------------------
void setup()
{
  // Start serial 1 as external control communication
  Serial1.begin(115200);
  // Start serial 2 as rf communication
  Serial2.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIS_SEN, INPUT);
  pinMode(MOTOR_1_A, INPUT_PULLDOWN);
  pinMode(MOTOR_1_B, INPUT_PULLDOWN);
  pinMode(MOTOR_2_A, INPUT);
  pinMode(MOTOR_2_B, INPUT);
  pinMode(MOTOR_3_A, INPUT);
  pinMode(MOTOR_3_B, INPUT);
  pinMode(MOTOR_4_A, INPUT_PULLDOWN);
  pinMode(MOTOR_4_B, INPUT_PULLDOWN);
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

  timer.setPrescaleFactor(9999);
  timer.setOverflow(9);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh();
  timer.resume();
  Serial1.println("===============ROBOT START================");
  Serial1.println("System information");
  Serial1.print("timer freq: ");
  Serial1.println(timer.getTimerClkFreq());
  Serial1.print("pwm resolution: ");
  Serial1.println(pwm_max_val);
  Serial1.print("max speed: ");
  Serial1.print(max_linear_velocity);
  Serial1.print(" m/s\tturn: ");
  Serial1.print(max_angular_velocity);
  Serial1.println(" rad/s");

  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  motor1.reset();
  motor2.reset();
  motor3.reset();
  motor4.reset();

  motor1.write(80, LOW);
  motor2.write(80, LOW);
  motor3.write(80, LOW);
  motor4.write(80, LOW);
}

void loop()
{
  const uint32_t cycle = 1000;
  const uint32_t t = millis();
  if (t - tick_t >= cycle)
  {
    Serial1.print("d_p: ");
    Serial1.print(motor1.d_p);
    Serial1.print("  ");
    Serial1.print(motor2.d_p);
    Serial1.print("  ");
    Serial1.print(motor3.d_p);
    Serial1.print("  ");
    Serial1.print(motor4.d_p);
    Serial1.print("\trpm: ");
    Serial1.print(motor1.speed());
    Serial1.print("  ");
    Serial1.print(motor2.speed());
    Serial1.print("  ");
    Serial1.print(motor3.speed());
    Serial1.print("  ");
    Serial1.print(motor4.speed());
    Serial1.print("\tm/s: ");
    Serial1.print(motor1.velocity());
    Serial1.print("  ");
    Serial1.print(motor2.velocity());
    Serial1.print("  ");
    Serial1.print(motor3.velocity());
    Serial1.print("  ");
    Serial1.print(motor4.velocity());
    Serial1.println('.');
    tick_t += cycle;
  }
}

// -------------------------------------------------DECLARE FUNCTIONS--------------------------------------------------------
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
    const double linear = doc["linear"];
    const double angular = doc["angular"];
    // Serial1.println("control velocity");
    // Serial1.print("linear: ");
    // Serial1.print(linear);
    // Serial1.print("\tangular: ");
    // Serial1.println(angular);
    velocityProcess(linear, angular);
  }
  else if (topic_name.compareTo("clear") == 0)
  {
    motor1.reset();
    motor2.reset();
    motor3.reset();
    motor4.reset();
  }
  else
  {
    //
  }
}

void velocityProcess(double linear, double angular)
{
  const double angular_tmp = angular * angular_velocity_factor;
  double linear_tmp = linear;
  const double motor_1_velocity = linear_tmp - angular_tmp;
  const double motor_2_velocity = linear_tmp - angular_tmp;
  const double motor_3_velocity = linear_tmp + angular_tmp;
  const double motor_4_velocity = linear_tmp + angular_tmp;
  Serial1.print("WHEEL VELOCITY: ");
  Serial1.print(motor_1_velocity);
  Serial1.print("\t");
  Serial1.print(motor_2_velocity);
  Serial1.print("\t");
  Serial1.print(motor_3_velocity);
  Serial1.print("\t");
  Serial1.print(motor_4_velocity);
  Serial1.print("\tWHEEL SPEED: ");
  Serial1.print(motor_1_velocity * wheel_speed_factor);
  Serial1.print("\t");
  Serial1.print(motor_2_velocity * wheel_speed_factor);
  Serial1.print("\t");
  Serial1.print(motor_3_velocity * wheel_speed_factor);
  Serial1.print("\t");
  Serial1.print(motor_4_velocity * wheel_speed_factor);
  Serial1.println("");
  motor1.drive(motor_1_velocity);
  motor2.drive(motor_2_velocity);
  motor3.drive(motor_3_velocity);
  motor4.drive(motor_4_velocity);
}