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
const double WHEEL_SEPARATION = 0.2; // m
const double WHEEL_DISTANCE = 0.146; // m
const double WHEEL_DIAMETER = 0.09;  // m
const double MOTOR_MAX_SPEED = 333;  // rpm
const double MOTOR_MAX_PPR = 1320.0; // motor max speed in pulses per round
const double ANGULAR_VELOCITY_FACTOR = (WHEEL_DISTANCE * WHEEL_DISTANCE + WHEEL_SEPARATION * WHEEL_SEPARATION) / (2 * WHEEL_SEPARATION);
const double WHEEL_SPEED_FACTOR = 60 / (3.1416 * WHEEL_DIAMETER);
const double MAX_LINEAR_VELOCITY = MOTOR_MAX_SPEED / WHEEL_SPEED_FACTOR;
const double MAX_ANGULAR_VELOCITY = (2 * MOTOR_MAX_SPEED) / (WHEEL_SPEED_FACTOR * WHEEL_SEPARATION);
const int32_t PWM_RESOLUTION_SET = 16;
const uint32_t PWM_MAX_VAL = pow(2, PWM_RESOLUTION_SET) - 1;
const uint32_t VEL_CAL_CYCLE = 20;     // in ms
const uint32_t PID_SAMPLE_CYCLE = 100; // in ms

class MotorControl
{
private:
#define DIRECT 0
#define REVERSE 1
  // Motor parameters
  String id;
  int64_t a;             // a channel count
  int64_t b;             // b channel count
  int64_t p_ins;         // instant encoder pulse value
  int64_t p_pre;         // previous encoder pulse value
  int64_t p_set;         // setpoint encoder pulse value
  int64_t p_ave;         // average encoder pulse value
  double p_kp = 2.0;     //
  double p_ki = 0;       //
  double p_kd = 0;       //
  double s;              //
  double v_ins;          // instant motor velocity in m/s
  double v_pre = 0;      // previous motor velocity in m/s
  double v_set;          // setpoint motor velocity in m/s
  double v_ave;          // average motor velocity in m/s
  uint32_t v_cou = 0;    // velocity sample counter
  double v_sum = 0;      // velocity sum
  double v_kp = 20.0;    //
  double v_ki = 1;       //
  double v_kd = 0;       //
  double v_Pro = 0;      // velocity proportional
  double v_Int = 0;      // velocity integral
  double v_Der = 0;      // velocity derivative
  double v_ei = 0;       // instant velocity error
  double v_de = 0;       //
  uint32_t last_t;       // last time call in milisecond
  uint32_t last_pid_t;   // last time call in milisecond
  uint32_t DIR_PIN_ADDR; // direction pin address
  uint32_t PWM_PIN_ADDR; // pwm pin address
  uint32_t A_PIN_ADDR;   // encoder channel A pin address
  uint32_t B_PIN_ADDR;   // encoder channel B pin address
  String state;
  uint32_t controlMode; // 1 for position control, 2 for velocity control
  uint32_t SampleTime;
  uint32_t controllerDirection;

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

    uint32_t duty = P_Block + I_Block + D_Block;
    if (err > 20)
      this->write(duty, HIGH);
    else if (err < -20)
      this->write(duty, LOW);
    else
      this->stop();
  }
  void velocityCompute()
  {
    this->v_Pro = this->v_set - this->v_ins;
    double pid_scale_factor = 1;
    // if (this->v_Pro < 0.1)
    //   pid_scale_factor = 2;
    // else if (this->v_Pro < 0.2)
    //   pid_scale_factor = 1.4;
    const double B_Voltage = this->v_set * WHEEL_SPEED_FACTOR * 12 / MOTOR_MAX_SPEED; // %
    double P_Voltage = pid_scale_factor * this->v_kp * this->v_Pro;
    double I_Voltage = pid_scale_factor * this->v_ki * this->v_Int;
    double D_Voltage = pid_scale_factor * this->v_kd * this->v_Der;
    this->v_Int = 0;
    this->v_Der = 0;

    uint32_t duty = abs(B_Voltage + P_Voltage + I_Voltage + D_Voltage) * PWM_MAX_VAL / 12;
    // Serial2.print(B_Voltage);
    // Serial2.print(" ");
    // Serial2.print(P_Voltage);
    // Serial2.print(" ");
    // Serial2.print(I_Voltage);
    // Serial2.print(" ");
    // Serial2.print(D_Voltage);
    // Serial2.print(" ");
    // Serial2.println(duty);
    if (duty > PWM_MAX_VAL)
      duty = PWM_MAX_VAL;
    if (duty < 0)
      duty = 0;
    if (B_Voltage > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, PWM_MAX_VAL - duty);
    }
    else
    {
      digitalWrite(this->DIR_PIN_ADDR, LOW);
      analogWrite(this->PWM_PIN_ADDR, duty);
    }
  }

public:
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
    Serial1.print("Motor ");
    Serial1.print(this->id);
    Serial1.print("\tVelocity:");
    Serial1.print(" kp:");
    Serial1.print(this->v_kp);
    Serial1.print(" ki:");
    Serial1.print(this->v_ki);
    Serial1.print(" kd:");
    Serial1.println(this->v_kd);
    Serial1.print("Position: ");
    Serial1.print(" kp:");
    Serial1.print(this->p_kp);
    Serial1.print(" ki:");
    Serial1.print(this->p_ki);
    Serial1.print(" kd:");
    Serial1.println(this->p_kd);
  }
  void tick(uint32_t t)
  {
    const int64_t present_p = this->p_ins;
    const uint32_t present_t = millis();
    uint32_t d_t = present_t - this->last_t;
    int64_t d_p = present_p - this->p_pre;
    this->v_ins = (d_p * 60000.0) / (d_t * MOTOR_MAX_PPR * WHEEL_SPEED_FACTOR);
    this->p_pre = present_p;
    this->last_t = present_t;
    this->v_sum += v_ins;
    this->v_cou++;
    this->v_Int += (this->v_ins - this->v_set) * d_t;
    this->v_Der += (this->v_ins - this->v_pre) / d_t;
    this->v_pre = this->v_ins;
    if (present_t - this->last_pid_t >= PID_SAMPLE_CYCLE)
    {
      this->last_pid_t = present_p;
      pidCompute();
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
  int32_t getA()
  {
    return this->a;
  }
  int32_t getB()
  {
    return this->b;
  }
  int32_t getPulses()
  {
    return this->p_ins;
  }
  void setPosition(int32_t p)
  {
    this->p_set = p;
    this->controlMode = 1;
  }
  void setVelocity(double v)
  {
    this->v_set = v;
    this->controlMode = 2;
  }
  void drive(double velocity)
  {
    const double wheel_speed = abs(velocity) * WHEEL_SPEED_FACTOR;        // rpm
    const uint32_t pwm_val = wheel_speed * PWM_MAX_VAL / MOTOR_MAX_SPEED; // %
    if (velocity > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, PWM_MAX_VAL - pwm_val);
    }
    else
    {
      digitalWrite(this->DIR_PIN_ADDR, LOW);
      analogWrite(this->PWM_PIN_ADDR, pwm_val);
    }
  }
  void rotate(double speed)
  {
    double wheel_speed = abs(speed); // rpm
    if (wheel_speed > MOTOR_MAX_SPEED)
      wheel_speed = MOTOR_MAX_SPEED;
    uint32_t pwm_val = wheel_speed * PWM_MAX_VAL / MOTOR_MAX_SPEED; // %
    if (speed > 0)
    {
      digitalWrite(this->DIR_PIN_ADDR, HIGH);
      analogWrite(this->PWM_PIN_ADDR, PWM_MAX_VAL - pwm_val);
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
    setPosition(this->p_ins);
  }
  void stop()
  {
    digitalWrite(this->DIR_PIN_ADDR, LOW);
    analogWrite(this->PWM_PIN_ADDR, LOW);
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
String messageFromSerial1 = "";
String messageFromSerial2 = "";

void msgProcess(String);
void velocityProcess(double, double);
void velocityProcess_base(double, double);

// -------------------------------------------------TIMER--------------------------------------------------------
HardwareTimer timer(TIM1);
uint64_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;
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
  // Start serial 1 as external control communication
  Serial1.begin(115200);
  // Start serial 2 as wireless communication (rf/bluetooth)
  Serial2.begin(115200);

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
  Serial1.println("===============ROBOT START================");
  Serial1.println("System information");
  Serial1.print("timer freq: ");
  Serial1.println(timer.getTimerClkFreq());
  Serial1.print("pwm resolution: ");
  Serial1.println(PWM_MAX_VAL);
  Serial1.print("max speed: ");
  Serial1.print(MAX_LINEAR_VELOCITY);
  Serial1.print(" m/s\tturn: ");
  Serial1.print(MAX_ANGULAR_VELOCITY);
  Serial1.println(" rad/s");

  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  motor1.reset();
  motor2.reset();
  motor3.reset();
  motor4.reset();
  // motor1.info();
  // motor1.info();
  // motor1.info();
  // motor1.info();

  // motor1.write(127, 0);
  // motor2.write(127, 0);
  // motor3.write(127, 0);
  // motor4.write(127, 0);

  // Serial2.println("AT");

  Serial1.println(SERIAL_UART_INSTANCE);
}

void loop()
{
  const uint32_t cycle = 50;
  const uint32_t t = millis();
  if (t - t_previous >= cycle)
  {
    // Serial1.print("p: ");
    // Serial1.print(motor1.getPulses());
    // Serial1.print("  ");
    // Serial1.print(motor2.getPulses());
    // Serial1.print("  ");
    // Serial1.print(motor3.getPulses());
    // Serial1.print("  ");
    // Serial1.print(motor4.getPulses());
    // Serial1.print("\ta: ");
    // Serial1.print(motor1.getA());
    // Serial1.print("  ");
    // Serial1.print(motor2.getA());
    // Serial1.print("  ");
    // Serial1.print(motor3.getA());
    // Serial1.print("  ");
    // Serial1.print(motor4.getA());
    // Serial1.print("\tb: ");
    // Serial1.print(motor1.getB());
    // Serial1.print("  ");
    // Serial1.print(motor2.getB());
    // Serial1.print("  ");
    // Serial1.print(motor3.getB());
    // Serial1.print("  ");
    // Serial1.print(motor4.getB());
    // Serial1.print("m/s: ");
    // Serial1.print("m1=");
    Serial2.print(motor1.getVelocity());
    Serial2.print(" ");
    Serial2.print(motor2.getVelocity());
    Serial2.print(" ");
    Serial2.print(motor3.getVelocity());
    Serial2.print(" ");
    Serial2.println(motor4.getVelocity());
    // Serial1.print("\trpm: ");
    // Serial1.print(motor1.getSpeed());
    // Serial1.print("  ");
    // Serial1.print(motor2.getSpeed());
    // Serial1.print("  ");
    // Serial1.print(motor3.getSpeed());
    // Serial1.print("  ");
    // Serial1.print(motor4.getSpeed());
    // Serial1.println('.');
    t_previous += cycle;
  }
  while (Serial2.available())
  {
    char tempChar = (char)Serial2.read();
    if (tempChar != '\n')
    {
      messageFromSerial2 += tempChar;
    }
    else
    {
      msgProcess(messageFromSerial2);
      Serial1.print("RF: ");
      Serial1.print(messageFromSerial2);
      messageFromSerial2 = "";
    }
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

void serialEvent2()
{
  while (Serial2.available())
  {
    char tempChar = (char)Serial2.read();
    Serial1.print(tempChar);
    if (tempChar != '\n')
    {
      messageFromSerial2 += tempChar;
    }
    else
    {
      Serial1.println(messageFromSerial2);
      messageFromSerial2 = "";
    }
  }
}

void msgProcess(String lightCmd)
{
  const uint8_t len = lightCmd.length();
  char json[len];
  lightCmd.toCharArray(json, len);
  // Serial1.println(json);
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
    const double linear_x = doc["linear"][0];   // x
    const double linear_y = doc["linear"][1];   // x
    const double linear_z = doc["linear"][2];   // x
    const double angular_r = doc["angular"][0]; // roll
    const double angular_p = doc["angular"][1]; // pitch
    const double angular_y = doc["angular"][2]; // yaw
    velocityProcess(linear_x, angular_y);
  }
  else if (topic_name.compareTo("base_control") == 0)
  {
    const double linear_x = doc["linear"][0];   // x
    const double linear_y = doc["linear"][1];   // x
    const double linear_z = doc["linear"][2];   // x
    const double angular_r = doc["angular"][0]; // roll
    const double angular_p = doc["angular"][1]; // pitch
    const double angular_y = doc["angular"][2]; // yaw
    velocityProcess_base(linear_x, angular_y);
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

void velocityProcess(double linear, double angular)
{
  const double angular_tmp = angular * ANGULAR_VELOCITY_FACTOR;
  double linear_tmp = linear;
  const double motor_1_velocity = linear_tmp + angular_tmp;
  const double motor_2_velocity = linear_tmp + angular_tmp;
  const double motor_3_velocity = linear_tmp - angular_tmp;
  const double motor_4_velocity = linear_tmp - angular_tmp;
  Serial1.print("=> m/s: ");
  Serial1.print(motor_1_velocity);
  Serial1.print("   ");
  Serial1.print(motor_2_velocity);
  Serial1.print("   ");
  Serial1.print(motor_3_velocity);
  Serial1.print("   ");
  Serial1.print(motor_4_velocity);
  Serial1.print("   rpm: ");
  Serial1.print(motor_1_velocity * WHEEL_SPEED_FACTOR);
  Serial1.print("   ");
  Serial1.print(motor_2_velocity * WHEEL_SPEED_FACTOR);
  Serial1.print("   ");
  Serial1.print(motor_3_velocity * WHEEL_SPEED_FACTOR);
  Serial1.print("   ");
  Serial1.print(motor_4_velocity * WHEEL_SPEED_FACTOR);
  Serial1.println("");
  motor1.setVelocity(motor_1_velocity);
  motor2.setVelocity(motor_2_velocity);
  motor3.setVelocity(motor_3_velocity);
  motor4.setVelocity(motor_4_velocity);
  Serial.println("");
}

void velocityProcess_base(double linear, double angular)
{
  const double angular_tmp = angular * ANGULAR_VELOCITY_FACTOR;
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
  Serial1.print(motor_1_velocity * WHEEL_SPEED_FACTOR);
  Serial1.print("\t");
  Serial1.print(motor_2_velocity * WHEEL_SPEED_FACTOR);
  Serial1.print("\t");
  Serial1.print(motor_3_velocity * WHEEL_SPEED_FACTOR);
  Serial1.print("\t");
  Serial1.print(motor_4_velocity * WHEEL_SPEED_FACTOR);
  Serial1.println("");
  // Serial.print("generate pwm: ");
  motor1.drive(motor_1_velocity);
  motor2.drive(motor_2_velocity);
  motor3.drive(motor_3_velocity);
  motor4.drive(motor_4_velocity);
  // Serial.println("");
}