#include <Arduino.h>
#include <ArduinoJson.h>
#include "checksum.h"
#include "hardware_control.h"
#include <math.h>

// -------------------------------------------------JSON--------------------------------------------------------
StaticJsonDocument<400> doc;

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

int64_t lastMsgProcess = 0;
void msgProcess(String, Stream &);
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
  if (timer_count % CONFIG.VEL_CAL_CYCLE == 0)
  {
    motor1.tick(CONFIG.VEL_CAL_CYCLE);
    motor2.tick(CONFIG.VEL_CAL_CYCLE);
    motor3.tick(CONFIG.VEL_CAL_CYCLE);
    motor4.tick(CONFIG.VEL_CAL_CYCLE);
  }
  if (timer_count % CONFIG.BAT_VOL_CAL_CYCLE == 0)
  {
    battery.tick();
  }
  if (timer_count % CONFIG.DIS_VOL_CAL_CYCLE == 0)
  {
    distance.tick();
  }
  if (timer_count % CONFIG.IMU_RAW_CAL_CYCLE == 0)
  {
    imu.tick();
  }
  else if (timer_count % 1000 == 0)
  {
    //
  }
}

// -------------------------------------------------MAIN CODE--------------------------------------------------------
void setup()
{
  analogWriteResolution(CONFIG.PWM_RESOLUTION_SET);
  // Start serial 2 as external control communication
  Bridge.begin(115200);
  // Start serial 1 as wireless communication (rf/bluetooth)
  Radio.begin(115200);
  // Serial.print("Start imu init....   ");
  if (imu.init(false) == 0)
    CONFIG.IMU_AVAILABLE = true;

  pinMode(DISTANCE_SENSOR, INPUT);
  pinMode(BATTERY_SENSOR, INPUT);
  pinMode(MOTOR_EN, OUTPUT);
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
  Bridge.println(CONFIG.PWM_MAX_VAL);
  Bridge.print("max speed: ");
  Bridge.print(CONFIG.MOTOR_MAX_SPEED_IN_MPS);
  Bridge.print(" m/s\tturn: ");
  Bridge.print(CONFIG.MAX_ANGULAR_VELOCITY);
  Bridge.println(" rad/s");

  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  delay(1000);
  motor1.reset();
  motor2.reset();
  motor3.reset();
  motor4.reset();

  enableMotor();
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

    // Bridge.print("battery: ");
    // Bridge.println(battery.getAverageVoltage());

    // Bridge.print("distance: ");
    // Bridge.println(distance.getVoltage());
    // Bridge.println(analogRead(PA1));

    if (CONFIG.EN_VELOCITY_LOG)
    {
      // Radio.print(motor1.getAverageSpeed());
      // Radio.print(" ");
      // Radio.print(motor2.getAverageSpeed());
      // Radio.print(" ");
      // Radio.print(motor3.getAverageSpeed());
      // Radio.print(" ");
      // Radio.print(motor4.getAverageSpeed());
      // Radio.print(" ");
      // Radio.print(motor1.getSetSpeed());
      // Radio.print(" ");
      // Radio.print(motor4.getSetSpeed());
      // Radio.println("");
      Radio.print("M1=");
      Radio.print(motor1.getAverageSpeed());
      Radio.print(",M2=");
      Radio.print(motor2.getAverageSpeed());
      Radio.print(",M3=");
      Radio.print(motor3.getAverageSpeed());
      Radio.print(",M4=");
      Radio.print(motor4.getAverageSpeed());
      Radio.print(",SR=");
      Radio.print(motor1.getSetSpeed());
      Radio.print(",SL=");
      Radio.print(motor4.getSetSpeed());
      Radio.print("\r\n");
    }
    else if (CONFIG.EN_IMU_LOG)
    {
      sensors sen = imu.getFullSensors();
      Radio.print(sen.orientation.x);
      Radio.print(" ");
      Radio.print(sen.orientation.y);
      Radio.print(" ");
      Radio.print(sen.orientation.z);
      Radio.print(" ");
      Radio.print(sen.orientation.w);
      Radio.print(" ");
      Radio.print(sen.angular_velocity.x);
      Radio.print(" ");
      Radio.print(sen.angular_velocity.y);
      Radio.print(" ");
      Radio.print(sen.angular_velocity.z);
      Radio.print(" ");
      Radio.print(sen.linear_acceleration.x);
      Radio.print(" ");
      Radio.print(sen.linear_acceleration.y);
      Radio.print(" ");
      Radio.print(sen.linear_acceleration.z);
      Radio.print(" ");
      Radio.print(sen.temperature);
      Radio.println("");
    }
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
      // Radio.println(messageFromBridge);
      msgProcess(messageFromBridge, Bridge);
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
      msgProcess(messageFromRadio, Radio);
      messageFromRadio = "";
    }
  }
}

void msgProcess(String lightCmd, Stream &stream)
{
  uint32_t idx = lightCmd.indexOf('{'); //

  String cmd_cs = lightCmd.substring(0, idx); // received checksum
  uint32_t rec_cs = 0;
  for (unsigned int i = 0; i < cmd_cs.length(); i++)
  {
    rec_cs = rec_cs * 10 + (cmd_cs[i] - '0');
  }
  lightCmd = lightCmd.substring(idx);       // split light comment from message
  uint32_t cal_cs = crc_generate(lightCmd); // calculated checksum

  // Radio.print("rec_cs: ");
  // Radio.println(rec_cs);
  // Radio.print("cal_cs: ");
  // Radio.println(cal_cs);

  // checksum
  if (CONFIG.CRC_Enable == true)
  {
    if (rec_cs != cal_cs)
      return;
  }
  const uint8_t len = lightCmd.length();
  char json[len];
  lightCmd.toCharArray(json, len);
  // Bridge.print(lightCmd);

  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    stream.print("deserializeJson() failed, error: ");
    stream.println(error.f_str());
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

    if (abs(angular_y) > 4.0 || abs(linear_x) > 0.4 || abs(linear_y) > 0.4)
    {
      doc["status"] = "invalid";
      char buffer[100];
      serializeJson(doc, buffer);
      String msg = String(buffer);
      msg = crc_generate(msg) + msg;
      stream.println(msg);
      return;
    }

    if (timeout == 0)
      velocity_timeout = CONFIG.DEFAULT_VEL_TIMEOUT;
    else
      velocity_timeout = timeout;
    velocityProcess(linear_x, linear_y, angular_y);
  }
  else if (topic_name.compareTo("ros2_control") == 0)
  {
    const double front_right_wheel_speed = doc["velocity"][0]; // rad/s
    const double rear_right_wheel_speed = doc["velocity"][1];  // rad/s
    const double rear_left_wheel_speed = doc["velocity"][2];   // rad/s
    const double front_left_wheel_speed = doc["velocity"][3];  // rad/s
    const uint32_t timeout = doc["timeout"];                   // ms
    const double motor_1_velocity = front_right_wheel_speed * CONFIG.WHEEL_DIAMETER / 2;
    const double motor_2_velocity = rear_right_wheel_speed * CONFIG.WHEEL_DIAMETER / 2;
    const double motor_3_velocity = rear_left_wheel_speed * CONFIG.WHEEL_DIAMETER / 2;
    const double motor_4_velocity = front_left_wheel_speed * CONFIG.WHEEL_DIAMETER / 2;
    if (timeout == 0)
      velocity_timeout = CONFIG.DEFAULT_VEL_TIMEOUT;
    else
      velocity_timeout = timeout;
    velocity_lastcall = millis();
    OnVelocityControl = true;
    if (CONFIG.EN_MECANUM_WHEEL == true)
    {
      motor1.setVelocity(motor_1_velocity);
      motor2.setVelocity(motor_2_velocity);
      motor3.setVelocity(motor_3_velocity);
      motor4.setVelocity(motor_4_velocity);
    }
    else
    {
      motor1.setVelocity(motor_1_velocity);
      motor2.setVelocity(motor_1_velocity);
      motor3.setVelocity(motor_4_velocity);
      motor4.setVelocity(motor_4_velocity);
    }
    // ros2 control response
    doc["status"] = "ok";
    char buffer[120];
    serializeJson(doc, buffer);
    String msg = String(buffer);
    msg = crc_generate(msg) + msg + "\r\n";
    stream.print(msg);
  }
  else if (topic_name.compareTo("ros2_state") == 0)
  {
    // add velocity value
    JsonArray velocity = doc.createNestedArray("vel");
    velocity.add(motor1.getAverageSpeed());
    velocity.add(motor2.getAverageSpeed());
    velocity.add(motor3.getAverageSpeed());
    velocity.add(motor4.getAverageSpeed());
    // add position value
    JsonArray position = doc.createNestedArray("pos");
    position.add(motor1.getPosition());
    position.add(motor2.getPosition());
    position.add(motor3.getPosition());
    position.add(motor4.getPosition());
    // add battery value
    doc["bat"] = battery.getAverageVoltage();
    // add imu value
    if (CONFIG.IMU_AVAILABLE)
    {
      sensors sen = imu.getFullSensors();
      // Serial.print("imu out: ");
      // Serial.print(sen.gyr.x);
      // Serial.print('\t');
      // Serial.print(sen.gyr.y);
      // Serial.print('\t');
      // Serial.print(sen.gyr.z);
      // Serial.print('\t');
      // Serial.print(sen.acc.x);
      // Serial.print('\t');
      // Serial.print(sen.acc.y);
      // Serial.print('\t');
      // Serial.print(sen.acc.z);
      // Serial.print('\t');
      // Serial.println(sen.tem);
      JsonArray orientation = doc.createNestedArray("ori");
      orientation.add(trimDouble(sen.orientation.x, 0));
      orientation.add(trimDouble(sen.orientation.y, 0));
      orientation.add(trimDouble(sen.orientation.z, 0));
      orientation.add(trimDouble(sen.orientation.w, 0));

      JsonArray gyroscope = doc.createNestedArray("gyr");
      gyroscope.add(trimDouble(sen.angular_velocity.x, 0));
      gyroscope.add(trimDouble(sen.angular_velocity.y, 0));
      gyroscope.add(trimDouble(sen.angular_velocity.z, 0));

      JsonArray accelerometer = doc.createNestedArray("acc");
      accelerometer.add(trimDouble(sen.linear_acceleration.x, 0));
      accelerometer.add(trimDouble(sen.linear_acceleration.y, 0));
      accelerometer.add(trimDouble(sen.linear_acceleration.z, 0));
      doc["tem"] = imu.getRawTemperature();
    }
    // add distance sensor value
    doc["dis"] = distance.getDistance();

    char buffer[400];
    serializeJson(doc, buffer);
    String msg = String(buffer);
    msg = crc_generate(msg) + msg + "\r\n";
    stream.print(msg);
  }
  else if (topic_name.compareTo("enableMotor") == 0)
  {
    enableMotor();
    stream.println("Succes Enable Motor!");
  }
  else if (topic_name.compareTo("disableMotor") == 0)
  {
    disableMotor();
    stream.println("Succes Disable Motor!");
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
  else if (topic_name.compareTo("config") == 0)
  {
    const String enableLog = doc["enable"];
    if (enableLog.compareTo("true") == 0)
      CONFIG.EN_VELOCITY_LOG = true;
    else if (enableLog.compareTo("false") == 0)
      CONFIG.EN_VELOCITY_LOG = false;
  }
  else if (topic_name.compareTo("enableVelocityLog") == 0)
  {
    CONFIG.EN_VELOCITY_LOG = true;
    stream.println("Succes Enable Velocity Log!");
  }
  else if (topic_name.compareTo("disableVelocityLog") == 0)
  {
    CONFIG.EN_VELOCITY_LOG = false;
    stream.println("Succes Disable Velocity Log!");
  }
  else if (topic_name.compareTo("enableCRC") == 0)
  {
    CONFIG.CRC_Enable = true;
    stream.println("Succes Enable CRC!");
  }
  else if (topic_name.compareTo("disableCRC") == 0)
  {
    CONFIG.CRC_Enable = false;
    stream.println("Success Disable CRC!");
  }
  else if (topic_name.compareTo("enableMecanum") == 0)
  {
    CONFIG.EN_MECANUM_WHEEL = true;
    stream.println("Succes Enable Mecanum Wheel Mode!");
  }
  else if (topic_name.compareTo("disableMecanum") == 0)
  {
    CONFIG.EN_MECANUM_WHEEL = false;
    stream.println("Success Disable Mecanum Wheel Mode!");
  }
  else if (topic_name.compareTo("info") == 0)
  {
    motor1.info(stream);
    motor2.info(stream);
    motor3.info(stream);
    motor4.info(stream);
    stream.print("battery voltage: ");
    stream.print(battery.getAverageVoltage());
    stream.println(" V");

    stream.print("CRC Enable: ");
    stream.println(CONFIG.CRC_Enable);

    stream.print("Mecanum Wheel Enable: ");
    stream.println(CONFIG.EN_MECANUM_WHEEL);
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
  lastMsgProcess = micros();
}

void velocityProcess(double linear_x, double linear_y, double angular)
{
  velocity_lastcall = millis();
  OnVelocityControl = true;
  double motor_1_velocity = 0;
  double motor_2_velocity = 0;
  double motor_3_velocity = 0;
  double motor_4_velocity = 0;
  const double angular_tmp = angular * CONFIG.ANGULAR_VELOCITY_FACTOR;
  if (CONFIG.EN_MECANUM_WHEEL == true)
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
  // Bridge.print("=> m/s: ");
  // Bridge.print(motor_1_velocity);
  // Bridge.print("   ");
  // Bridge.print(motor_2_velocity);
  // Bridge.print("   ");
  // Bridge.print(motor_3_velocity);
  // Bridge.print("   ");
  // Bridge.print(motor_4_velocity);
  // Bridge.print("   rpm: ");
  // Bridge.print(motor_1_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  // Bridge.print("   ");
  // Bridge.print(motor_2_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  // Bridge.print("   ");
  // Bridge.print(motor_3_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  // Bridge.print("   ");
  // Bridge.print(motor_4_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  // Bridge.println("");
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
  const double angular_tmp = angular * CONFIG.ANGULAR_VELOCITY_FACTOR;
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
  Bridge.print(motor_1_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  Bridge.print("\t");
  Bridge.print(motor_2_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  Bridge.print("\t");
  Bridge.print(motor_3_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  Bridge.print("\t");
  Bridge.print(motor_4_velocity * CONFIG.MPS_TO_RPM_FACTOR);
  Bridge.println("");
}
