#include <Arduino.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <math.h>

// // define motor 1
// #define MOTOR_1_A PB3
// #define MOTOR_1_B PA15
// #define MOTOR_1_PWM PB8
// #define MOTOR_1_DIR PB9
// // define motor 2
// #define MOTOR_2_A PA12
// #define MOTOR_2_B PA11
// #define MOTOR_2_PWM PB4
// #define MOTOR_2_DIR PB5
// // define motor 3
// #define MOTOR_3_A PA6
// #define MOTOR_3_B PA7
// #define MOTOR_3_PWM PB1
// #define MOTOR_3_DIR PB0
// // define motor 4
// #define MOTOR_4_A PC13
// #define MOTOR_4_B PA0
// #define MOTOR_4_PWM PA5
// #define MOTOR_4_DIR PA4

// define motor 1
#define MOTOR_1_A PA7
#define MOTOR_1_B PA6
#define MOTOR_1_PWM PB10
#define MOTOR_1_DIR PB1
// define motor 2
#define MOTOR_2_A PA5
#define MOTOR_2_B PA4
#define MOTOR_2_PWM PB4
#define MOTOR_2_DIR PB5
// define motor 3
#define MOTOR_3_B PA15
#define MOTOR_3_A PB3
#define MOTOR_3_PWM PB15
#define MOTOR_3_DIR PA8
// define motor 4
#define MOTOR_4_B PA11
#define MOTOR_4_A PA12
#define MOTOR_4_PWM PB13
#define MOTOR_4_DIR PB14

// define sensor
#define DIS_SEN PA1

#define FRONT_LIGHT PC13

#define BATTERY_SENSOR PB0

// -------------------------------------------------JSON--------------------------------------------------------
StaticJsonDocument<200> doc;

// -------------------------------------------------SERIAL--------------------------------------------------------
HardwareSerial Bridge(PA3, PA2);
#define Radio Serial1
bool logVelocityEnable = false; //
bool logPidEnable = false;      //
String logMotorId = "";         // "M-1"|"M-2"|"M-3"|"M-4"

// -------------------------------------------------MOTOR--------------------------------------------------------
bool En_Mecanum_Wheel = true;
const double WHEEL_SEPARATION = 0.2; // m
const double WHEEL_DISTANCE = 0.146; // m
const double WHEEL_DIAMETER = 0.096; // m
const double MOTOR_MAX_SPEED = 111;  // rpm
const double MOTOR_MAX_PPR = 3960;   // motor max speed in pulses per round
const double ANGULAR_VELOCITY_FACTOR = (WHEEL_DISTANCE * WHEEL_DISTANCE + WHEEL_SEPARATION * WHEEL_SEPARATION) / (2 * WHEEL_SEPARATION);
const double WHEEL_SPEED_FACTOR = 60 / (M_PI * WHEEL_DIAMETER);
const double MAX_LINEAR_VELOCITY = MOTOR_MAX_SPEED / WHEEL_SPEED_FACTOR;
const double MAX_ANGULAR_VELOCITY = (2 * MOTOR_MAX_SPEED) / (WHEEL_SPEED_FACTOR * WHEEL_SEPARATION);
const int32_t PWM_RESOLUTION_SET = 16;
const uint32_t PWM_MAX_VAL = pow(2, PWM_RESOLUTION_SET) - 1;
const uint32_t VEL_CAL_CYCLE = 10;    // in ms
const uint32_t PID_SAMPLE_CYCLE = 10; // in ms

class MotorControl
{
private:
#define DIRECT 0
#define REVERSE 1
  // Motor parameters
  String id;
  int64_t a;              // a channel count
  int64_t b;              // b channel count
  int64_t p_ins;          // instant encoder pulse value
  int64_t p_pre;          // previous encoder pulse value
  int64_t p_set;          // setpoint encoder pulse value
  int64_t d_p = 0;        //
  int64_t p_ave;          // average encoder pulse value
  double p_kp = 0;        //
  double p_ki = 0;        //
  double p_kd = 0;        //
  double v_ins;           // instant motor velocity in m/s
  double v_pre = 0;       // previous motor velocity in m/s
  double v_set = 0;       // setpoint motor velocity in m/s
  double v_ave;           // average motor velocity in m/s
  uint32_t v_cou = 0;     // velocity sample counter
  double v_sum = 0;       // velocity sum
  double v_kp = 3.0;      //
  double v_ki = 0;        //
  double v_kd = 180.0;    //
  double v_Pro = 0;       // velocity proportional
  double v_Int = 0;       // velocity integral
  double v_Der = 0;       // velocity derivative
  double v_ei = 0;        // instant velocity error
  double v_de = 0;        //
  uint32_t timeout = 500; // velocity timeout, after this velocity set to 0
  uint32_t lastcall = 0;  // last time set velocity
  uint32_t last_t;        // last time call in milisecond
  uint32_t last_pid_t;    // last time call in milisecond
  uint32_t DIR_PIN_ADDR;  // direction pin address
  uint32_t PWM_PIN_ADDR;  // pwm pin address
  uint32_t A_PIN_ADDR;    // encoder channel A pin address
  uint32_t B_PIN_ADDR;    // encoder channel B pin address
  String state;           //
  uint32_t controlMode;   // 1 for position control, 2 for velocity control
  uint32_t SampleTime;
  uint32_t controllerDirection;
  uint32_t direction_factor = 1;

  struct pid_var
  {
    double kp, ki, kd;
    double outMax, outMin;
    uint32_t last_t;
  } position, velocity;
  void positionCompute()
  {
    double err = this->p_set - this->p_ins;

    double P_Block = this->p_kp * err;
    double I_Block = this->p_ki * 0;
    double D_Block = this->p_kd * 0;

    uint32_t Sum_Voltage = P_Block + I_Block + D_Block;
    this->drive(Sum_Voltage);
  }
  void velocityCompute()
  {
    this->v_Pro = this->v_set - this->v_ins;
    double pid_scale_factor = 1;
    if (this->v_Pro < 0.1)
      pid_scale_factor = 2;
    else if (this->v_Pro < 0.2)
      pid_scale_factor = 1.5;
    this->B_Voltage = this->voltage;
    // if (abs(this->v_Pro) > 0.005)
    //   this->P_Voltage = pid_scale_factor * this->v_kp * this->v_Pro;
    // else
    //   this->P_Voltage = 0;
    this->P_Voltage = pid_scale_factor * this->v_kp * this->v_Pro;
    this->I_Voltage = this->v_ki * this->v_Int;
    this->D_Voltage = this->v_kd * this->v_Der;

    this->v_Int = 0;
    this->v_Der = 0;

    double Sum_Voltage = B_Voltage + P_Voltage + I_Voltage + D_Voltage;
    this->drive(Sum_Voltage);
  }
  void drive(double set_voltage)
  {
    double output_voltage = set_voltage;
    if (output_voltage > 12)
      output_voltage = 12;
    if (output_voltage < -12)
      output_voltage = 12;
    uint32_t duty = abs(output_voltage) * PWM_MAX_VAL / 12;
    this->voltage = output_voltage;
    if (output_voltage > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, LOW);
      analogWrite(this->PWM_PIN_ADDR, duty);
    }
    else
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, PWM_MAX_VAL - duty);
    }
  }

public:
  double voltage; //
  double B_Voltage = 0;
  double P_Voltage = 0;
  double I_Voltage = 0;
  double D_Voltage = 0;

  MotorControl(String id, uint32_t direction_pin, uint32_t pwm_pin, uint32_t a_pin, uint32_t b_pin)
  {
    this->id = id;
    this->a = 0;
    this->b = 0;
    this->p_ins = 0;
    this->p_pre = 0;
    this->last_t = millis();
    this->DIR_PIN_ADDR = direction_pin;
    this->PWM_PIN_ADDR = pwm_pin;
    this->state = "float";
    this->controlMode = 1;
    if (this->id.compareTo("M-3") == 0 || this->id.compareTo("M-4") == 0)
      this->direction_factor = -1;
  }
  void setVelocityPID(double v_kp, double v_ki, double v_kd)
  {
    this->v_kp = v_kp;
    this->v_ki = v_ki;
    this->v_kd = v_kd;
  }
  void setPositionPID(double p_kp, double p_ki, double p_kd)
  {
    this->p_kp = p_kp;
    this->p_ki = p_ki;
    this->p_kd = p_kd;
  }
  void info()
  {
    Bridge.print("Motor ");
    Bridge.print(this->id);
    Bridge.print("\tVelocity:");
    Bridge.print(" kp:");
    Bridge.print(this->v_kp);
    Bridge.print(" ki:");
    Bridge.print(this->v_ki);
    Bridge.print(" kd:");
    Bridge.println(this->v_kd);
    Bridge.print("Position: ");
    Bridge.print(" kp:");
    Bridge.print(this->p_kp);
    Bridge.print(" ki:");
    Bridge.print(this->p_ki);
    Bridge.print(" kd:");
    Bridge.println(this->p_kd);
  }
  void tick(uint32_t t)
  {
    // calculate instant velocity
    const int64_t present_p = this->p_ins;
    const uint32_t present_t = millis();
    uint32_t d_t = present_t - this->last_t; // in miliseconds
    this->d_p = present_p - this->p_pre;
    this->v_ins = (this->d_p * 60000.0) / (d_t * MOTOR_MAX_PPR * WHEEL_SPEED_FACTOR);
    this->p_pre = present_p;
    this->last_t = present_t;

    // prepare for calculating everage velocity
    this->v_sum += v_ins;
    this->v_cou++;

    // calculate Integral
    this->v_Int += (this->v_set - this->v_ins) * d_t;
    // calculate derivative
    this->v_Der = (this->v_pre - this->v_ins) / d_t;

    this->v_pre = this->v_ins;

    // check pid cycle
    if (present_t - this->last_pid_t >= PID_SAMPLE_CYCLE)
    {
      this->last_pid_t = present_p;
      // pid computing
      pidCompute();
      // calculate average volocity
      this->v_ave = this->v_sum / this->v_cou;
      this->v_sum = 0;
      this->v_cou = 0;
    }
  }
  void pidCompute()
  {
    switch (this->controlMode)
    {
    case 1: // Control position mode
      this->positionCompute();
      break;
    case 2: // Control velocity mode
      this->velocityCompute();
      break;
    default:
      break;
    }
  }
  void step_fw(int32_t c)
  {
    this->a += c;
    this->p_ins += c;
  }
  void step_bw(int32_t c)
  {
    this->b += c;
    this->p_ins += c;
  }
  void reset()
  {
    this->a = 0;
    this->b = 0;
    this->p_ins = 0;
  }
  int64_t getPosition()
  {
    return this->p_ins;
  }
  double getVelocity()
  {
    return this->v_ins;
  }
  double getAverageVelocity()
  {
    return this->v_ave;
  }
  double getSpeed()
  {
    return this->v_ins * WHEEL_SPEED_FACTOR;
  }
  int32_t getdp()
  {
    return this->d_p;
  }
  void setPosition(int32_t p)
  {
    this->p_set = p;
    this->controlMode = 1;
  }
  void setVelocity(double v)
  {
    this->v_set = v;
    this->voltage = this->v_set * WHEEL_SPEED_FACTOR * 12 / MOTOR_MAX_SPEED;
    this->controlMode = 2;
    this->lastcall = millis();
  }
  void lock()
  {
    setPosition(this->p_ins);
  }
  void stop()
  {
    this->drive(0.0);
    this->controlMode = 0;
  }
} motor1("M-1", MOTOR_1_DIR, MOTOR_1_PWM, MOTOR_1_A, MOTOR_1_B),
    motor2("M-2", MOTOR_2_DIR, MOTOR_2_PWM, MOTOR_2_A, MOTOR_2_B),
    motor3("M-3", MOTOR_3_DIR, MOTOR_3_PWM, MOTOR_3_A, MOTOR_3_B),
    motor4("M-4", MOTOR_4_DIR, MOTOR_4_PWM, MOTOR_4_A, MOTOR_4_B);

void EncoderHandle_1_A()
{
  uint32_t state_a = digitalRead(MOTOR_1_A);
  uint32_t state_b = digitalRead(MOTOR_1_B);
  if (state_a == state_b)
    motor1.step_fw(1);
  else
    motor1.step_fw(-1);
}
void EncoderHandle_1_B()
{
  uint32_t state_a = digitalRead(MOTOR_1_A);
  uint32_t state_b = digitalRead(MOTOR_1_B);
  if (state_a == state_b)
    motor1.step_bw(-1);
  else
    motor1.step_bw(1);
}
void EncoderHandle_2_A()
{
  uint32_t state_a = digitalRead(MOTOR_2_A);
  uint32_t state_b = digitalRead(MOTOR_2_B);
  if (state_a == state_b)
    motor2.step_fw(1);
  else
    motor2.step_fw(-1);
}
void EncoderHandle_2_B()
{
  uint32_t state_a = digitalRead(MOTOR_2_A);
  uint32_t state_b = digitalRead(MOTOR_2_B);
  if (state_a == state_b)
    motor2.step_bw(-1);
  else
    motor2.step_bw(1);
}
void EncoderHandle_3_A()
{
  uint32_t state_a = digitalRead(MOTOR_3_A);
  uint32_t state_b = digitalRead(MOTOR_3_B);
  if (state_a == state_b)
    motor3.step_fw(1);
  else
    motor3.step_fw(-1);
}
void EncoderHandle_3_B()
{
  uint32_t state_a = digitalRead(MOTOR_3_A);
  uint32_t state_b = digitalRead(MOTOR_3_B);
  if (state_a == state_b)
    motor3.step_bw(-1);
  else
    motor3.step_bw(1);
}
void EncoderHandle_4_A()
{
  uint32_t state_a = digitalRead(MOTOR_4_A);
  uint32_t state_b = digitalRead(MOTOR_4_B);
  if (state_a == state_b)
    motor4.step_fw(1);
  else
    motor4.step_fw(-1);
}
void EncoderHandle_4_B()
{
  uint32_t state_a = digitalRead(MOTOR_4_A);
  uint32_t state_b = digitalRead(MOTOR_4_B);
  if (state_a == state_b)
    motor4.step_bw(-1);
  else
    motor4.step_bw(1);
}

// -------------------------------------------------PID CONTROLLER--------------------------------------------------------
uint32_t t_previous = millis();
String messageFromRadio = "";
String messageFromBridge = "";
uint32_t velocity_lastcall = 0;
uint32_t velocity_timeout = 500;
bool OnVelocityControl = false;

void msgProcess(String);
void velocityProcess(double, double, double);
void velocityProcessTimeout(double, double, double, uint32_t);
void velocityProcess_base(double, double, double);

// -------------------------------------------------TIMER--------------------------------------------------------
HardwareTimer timer(TIM1);
uint64_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;
  uint32_t present_t = millis();
  if (OnVelocityControl == true && present_t - velocity_lastcall >= velocity_timeout)
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    OnVelocityControl = false;
  }
  if (timer_count % VEL_CAL_CYCLE == 0)
  {
    motor1.tick(VEL_CAL_CYCLE);
    motor2.tick(VEL_CAL_CYCLE);
    motor3.tick(VEL_CAL_CYCLE);
    motor4.tick(VEL_CAL_CYCLE);
  }
  else if (timer_count % 1000 == 0)
  {
    //
  }
}

// -------------------------------------------------MAIN CODE--------------------------------------------------------
void setup()
{
  analogWriteResolution(PWM_RESOLUTION_SET);
  // Start serial 2 as external control communication
  Bridge.begin(115200);
  // Start serial 1 as wireless communication (rf/bluetooth)
  Radio.begin(115200);

  pinMode(DIS_SEN, INPUT);
  pinMode(MOTOR_1_A, INPUT);
  pinMode(MOTOR_1_B, INPUT);
  pinMode(MOTOR_2_A, INPUT);
  pinMode(MOTOR_2_B, INPUT);
  pinMode(MOTOR_3_A, INPUT);
  pinMode(MOTOR_3_B, INPUT);
  pinMode(MOTOR_4_A, INPUT);
  pinMode(MOTOR_4_B, INPUT);
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  pinMode(MOTOR_3_PWM, OUTPUT);
  pinMode(MOTOR_4_PWM, OUTPUT);
  pinMode(MOTOR_1_DIR, OUTPUT);
  pinMode(MOTOR_2_DIR, OUTPUT);
  pinMode(MOTOR_3_DIR, OUTPUT);
  pinMode(MOTOR_4_DIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_A), EncoderHandle_1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_B), EncoderHandle_1_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_A), EncoderHandle_2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_B), EncoderHandle_2_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_A), EncoderHandle_3_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_B), EncoderHandle_3_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_A), EncoderHandle_4_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_B), EncoderHandle_4_B, CHANGE);

  timer.setPrescaleFactor(9999);
  timer.setOverflow(9);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh();
  timer.resume();
  Bridge.println("===============ROBOT START================");
  Bridge.println("System information");
  Bridge.print("timer freq: ");
  Bridge.println(timer.getTimerClkFreq());
  Bridge.print("pwm resolution: ");
  Bridge.println(PWM_MAX_VAL);
  Bridge.print("max speed: ");
  Bridge.print(MAX_LINEAR_VELOCITY);
  Bridge.print(" m/s\tturn: ");
  Bridge.print(MAX_ANGULAR_VELOCITY);
  Bridge.println(" rad/s");

  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  motor1.reset();
  motor2.reset();
  motor3.reset();
  motor4.reset();
}

void loop()
{
  const uint32_t cycle = 20;
  const uint32_t t = millis();
  if (t - t_previous >= cycle)
  {
    // Radio.print(motor1.voltage);
    // Radio.print(" ");
    // Radio.print(motor2.voltage);
    // Radio.print(" ");
    // Radio.print(motor3.voltage);
    // Radio.print(" ");
    // Radio.print(motor4.voltage);

    // Radio.print(motor3.B_Voltage);
    // Radio.print(" ");
    // Radio.print(motor3.P_Voltage);
    // Radio.print(" ");
    // Radio.print(motor3.voltage);
    // Radio.print(" ");
    // Radio.print(motor3.getVelocity());

    Radio.print(motor1.getVelocity());
    Radio.print(" ");
    Radio.print(motor2.getVelocity());
    Radio.print(" ");
    Radio.print(motor3.getVelocity());
    Radio.print(" ");
    Radio.print(motor4.getVelocity());

    Radio.println("");
    t_previous += cycle;
  }
  while (Bridge.available())
  {
    char tempChar = (char)Bridge.read();
    if (tempChar != '\n')
    {
      messageFromBridge += tempChar;
    }
    else
    {
      msgProcess(messageFromBridge);
      messageFromBridge = "";
    }
  }
}

// -------------------------------------------------DECLARE FUNCTIONS--------------------------------------------------------
void serialEvent1()
{
  while (Radio.available())
  {
    char tempChar = (char)Radio.read();
    if (tempChar != '\n')
    {
      messageFromRadio += tempChar;
    }
    else
    {
      msgProcess(messageFromRadio);
      messageFromRadio = "";
    }
  }
}

void msgProcess(String lightCmd)
{
  const uint8_t len = lightCmd.length();
  char json[len];
  lightCmd.toCharArray(json, len);
  Bridge.print(lightCmd);
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Bridge.print(F("deserializeJson() failed, message: "));
    Bridge.print(lightCmd);
    Bridge.print(" error: ");
    Bridge.println(error.f_str());
    return;
  }
  const char *topic = doc["topic"];
  const String topic_name = String(topic);
  if (topic_name.compareTo("control") == 0)
  {
    const double linear_x = doc["linear"][0];   // x
    const double linear_y = doc["linear"][1];   // y
    const double linear_z = doc["linear"][2];   // z
    const double angular_r = doc["angular"][0]; // roll
    const double angular_p = doc["angular"][1]; // pitch
    const double angular_y = doc["angular"][2]; // yaw
    const uint32_t timeout = doc["timeout"];    // timeout
    Bridge.print("timeout: ");
    Bridge.println(timeout);
    if (timeout == 0)
      velocityProcess(linear_x, linear_y, angular_y);
    else
      velocityProcessTimeout(linear_x, linear_y, angular_y, timeout);
  }
  else if (topic_name.compareTo("base_control") == 0)
  {
    const double linear_x = doc["linear"][0];   // x
    const double linear_y = doc["linear"][1];   // x
    const double linear_z = doc["linear"][2];   // x
    const double angular_r = doc["angular"][0]; // roll
    const double angular_p = doc["angular"][1]; // pitch
    const double angular_y = doc["angular"][2]; // yaw
    velocityProcess_base(linear_x, linear_y, angular_y);
  }
  else if (topic_name.compareTo("configPID") == 0)
  {
    const String mode = doc["mode"];
    const double kp = doc["kp"];
    const double ki = doc["ki"];
    const double kd = doc["kd"];
    if (mode.compareTo("velocity") == 0)
    {
      motor1.setVelocityPID(kp, ki, kd);
      motor2.setVelocityPID(kp, ki, kd);
      motor3.setVelocityPID(kp, ki, kd);
      motor4.setVelocityPID(kp, ki, kd);
    }
    else if (mode.compareTo("position") == 0)
    {
      motor1.setPositionPID(kp, ki, kd);
      motor2.setPositionPID(kp, ki, kd);
      motor3.setPositionPID(kp, ki, kd);
      motor4.setPositionPID(kp, ki, kd);
    }
  }
  else if (topic_name.compareTo("configLog") == 0)
  {
    bool velocityLogEnable = doc["velocity"];
    bool pidLogEnable = doc["velocity"];
    if (velocityLogEnable == true)
      logVelocityEnable = true;
    else
      logVelocityEnable = false;
    if (pidLogEnable == true)
    {
      logVelocityEnable = true;
    }
  }
  else if (topic_name.compareTo("info") == 0)
  {
    motor1.info();
    motor2.info();
    motor3.info();
    motor4.info();
  }
  else if (topic_name.compareTo("stop") == 0)
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
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

void velocityProcess(double linear_x, double linear_y, double angular)
{
  velocity_lastcall = millis();
  OnVelocityControl = true;
  double motor_1_velocity = 0;
  double motor_2_velocity = 0;
  double motor_3_velocity = 0;
  double motor_4_velocity = 0;
  const double angular_tmp = angular * ANGULAR_VELOCITY_FACTOR;
  if (En_Mecanum_Wheel == true)
  {
    motor_1_velocity = linear_x + linear_y + angular_tmp;
    motor_2_velocity = linear_x - linear_y + angular_tmp;
    motor_3_velocity = linear_x + linear_y - angular_tmp;
    motor_4_velocity = linear_x - linear_y - angular_tmp;
  }
  else
  {
    motor_1_velocity = linear_x + angular_tmp;
    motor_2_velocity = linear_x + angular_tmp;
    motor_3_velocity = linear_x - angular_tmp;
    motor_4_velocity = linear_x - angular_tmp;
  }
  Bridge.print("=> m/s: ");
  Bridge.print(motor_1_velocity);
  Bridge.print("   ");
  Bridge.print(motor_2_velocity);
  Bridge.print("   ");
  Bridge.print(motor_3_velocity);
  Bridge.print("   ");
  Bridge.print(motor_4_velocity);
  Bridge.print("   rpm: ");
  Bridge.print(motor_1_velocity * WHEEL_SPEED_FACTOR);
  Bridge.print("   ");
  Bridge.print(motor_2_velocity * WHEEL_SPEED_FACTOR);
  Bridge.print("   ");
  Bridge.print(motor_3_velocity * WHEEL_SPEED_FACTOR);
  Bridge.print("   ");
  Bridge.print(motor_4_velocity * WHEEL_SPEED_FACTOR);
  Bridge.println("");
  motor1.setVelocity(motor_1_velocity);
  motor2.setVelocity(motor_2_velocity);
  motor3.setVelocity(motor_3_velocity);
  motor4.setVelocity(motor_4_velocity);
}

void velocityProcessTimeout(double linear_x, double linear_y, double angular, uint32_t timeout)
{
  velocity_timeout = timeout;
  velocityProcess(linear_x, linear_y, angular);
}

void velocityProcess_base(double linear_x, double linear_y, double angular)
{
  velocity_lastcall = millis();
  const double angular_tmp = angular * ANGULAR_VELOCITY_FACTOR;
  double linear_tmp = linear_x;
  const double motor_1_velocity = linear_tmp + angular_tmp;
  const double motor_2_velocity = linear_tmp + angular_tmp;
  const double motor_3_velocity = linear_tmp - angular_tmp;
  const double motor_4_velocity = linear_tmp - angular_tmp;
  Bridge.print("WHEEL VELOCITY: ");
  Bridge.print(motor_1_velocity);
  Bridge.print("\t");
  Bridge.print(motor_2_velocity);
  Bridge.print("\t");
  Bridge.print(motor_3_velocity);
  Bridge.print("\t");
  Bridge.print(motor_4_velocity);
  Bridge.print("\tWHEEL SPEED: ");
  Bridge.print(motor_1_velocity * WHEEL_SPEED_FACTOR);
  Bridge.print("\t");
  Bridge.print(motor_2_velocity * WHEEL_SPEED_FACTOR);
  Bridge.print("\t");
  Bridge.print(motor_3_velocity * WHEEL_SPEED_FACTOR);
  Bridge.print("\t");
  Bridge.print(motor_4_velocity * WHEEL_SPEED_FACTOR);
  Bridge.println("");
}