#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include <Adafruit_INA219.h>
#include "checksum.h"
#include "hardware_control.h"
#include "currentSensor.hpp"
#include "lcd.hpp"

// -------------------------------------------------JSON--------------------------------------------------------
StaticJsonDocument<500> doc;

// -------------------------------------------------INA219--------------------------------------------------------
Adafruit_INA219 ina219(0x45);

// -------------------------------------------------PID CONTROLLER--------------------------------------------------------
String messageFromRadio = "";
String messageFromBridge = "";
uint32_t velocity_lastcall_ms = 0;
uint32_t velocity_timeout_ms = 500;
bool OnVelocityControl = false;
float vol = 0;
float mAmp = 0;
double mW_sum = 0;
double mA_sum = 0;
uint8_t mW_count = 0;
uint8_t mA_count = 0;
double POWER_RAW_TO_MILI_WATT_HOUR = 0.1 / 3600;
double POWER_RAW_TO_WATT_HOUR = POWER_RAW_TO_MILI_WATT_HOUR / 1000;

void msgProcess(String, Stream &);
void logger();

// -------------------------------------------------MAIN CODE--------------------------------------------------------
CurrentSensor currentSensor_1_2(CURRENT_SENSOR_1_2),
    currentSensor_3_4(CURRENT_SENSOR_3_4);

// -------------------------------------------------TIMER--------------------------------------------------------
HardwareTimer timer(TIM1);
uint64_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;

  if (Radio.available())
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
  if (timer_count >= velocity_timeout_ms)
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }
  if (timer_count % MOTOR_CYCLE == 0)
  {
    motor1.tick(MOTOR_CYCLE);
    motor2.tick(MOTOR_CYCLE);
    motor3.tick(MOTOR_CYCLE);
    motor4.tick(MOTOR_CYCLE);
  }
  // switch (timer_count % MOTOR_CYCLE)
  // {
  // case 0:
  //   motor1.tick(MOTOR_CYCLE);
  //   break;
  // case 1:
  //   motor2.tick(MOTOR_CYCLE);
  //   break;
  // case 2:
  //   motor3.tick(MOTOR_CYCLE);
  //   break;
  // case 3:
  //   motor4.tick(MOTOR_CYCLE);
  //   break;

  // default:
  //   break;
  // }
  if (timer_count % CURRENT_CAL_CYCLE == 2)
  {
    currentSensor_1_2.tick();
    currentSensor_3_4.tick();
  }
  if (timer_count % BAT_VOL_CAL_CYCLE == 3 && POWER_METTER_AVAILABLE)
  {
    battery.tick();
    vol = ina219.getBusVoltage_V();
    mW_sum += ina219.getPower_mW() * 10;
    mAmp = ina219.getCurrent_mA() * 10;
    if (mAmp != infinityf())
      mA_sum += mAmp;
  }
  if (timer_count % 1000 == 4)
  {
    toggleLed();
  }
}

// -------------------------------------------------MAIN CODE--------------------------------------------------------
void setup()
{
  pinMode(BATTERY_SENSOR, INPUT);
  pinMode(CURRENT_SENSOR_1_2, INPUT);
  pinMode(CURRENT_SENSOR_3_4, INPUT);
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
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_A), EncoderHandle_1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_B), EncoderHandle_1_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_A), EncoderHandle_2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_B), EncoderHandle_2_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_A), EncoderHandle_3_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_B), EncoderHandle_3_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_A), EncoderHandle_4_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_B), EncoderHandle_4_B, CHANGE);

  timer.setPrescaleFactor(10000);
  timer.setOverflow(10);
  timer.attachInterrupt(OnTimer1Interrupt);

  analogWriteResolution(PWM_RESOLUTION_SET);
  // Start serial 1 as ros2 bridge
  Bridge.begin(460800);
  // Start serial 1 as debug port (rf/bluetooth)
  Radio.begin(115200);

  // Bridge.println("===> Setting up power manager");
  // dfMeter.init(Bridge);
  // dfMeter.info(Bridge);

  Bridge.println("===> Robot start");
  Bridge.println("System information");
  Bridge.print("timer freq: ");
  Bridge.println(timer.getTimerClkFreq());
  Bridge.print("pwm resolution: ");
  Bridge.println(PWM_MAX_VAL);
  Bridge.print("max speed: ");
  Bridge.print(MOTOR_MAX_SPEED_IN_MPS);
  Bridge.print(" m/s\tturn: ");
  Bridge.print(MAX_ANGULAR_VELOCITY);
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
  delay(100);
  currentSensor_1_2.calibrate();
  currentSensor_3_4.calibrate();

  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
  }
  else
    POWER_METTER_AVAILABLE = true;

  Bridge.println("================================setup complete========================================");

  timer.refresh();
  timer.resume();
}

void loop()
{
}

// -------------------------------------------------DECLARE FUNCTIONS--------------------------------------------------------
void bridgeEvent()
{
  messageFromBridge = Bridge.readStringUntil('\n');
  msgProcess(messageFromBridge, Bridge);
  messageFromBridge = "";
}

void msgProcess(String rawMessage, Stream &stream)
{
  uint64_t start_t = micros();

  // split string
  uint32_t idx = rawMessage.indexOf('{');              //
  String checksumBlock = rawMessage.substring(0, idx); // received checksum
  uint32_t rec_cs = 0;
  for (unsigned int i = 0; i < checksumBlock.length(); i++)
  {
    rec_cs = rec_cs * 10 + (checksumBlock[i] - '0');
  }
  rawMessage = rawMessage.substring(idx);     // split light comment from message
  uint32_t cal_cs = crc_generate(rawMessage); // calculated checksum

  // checksum
  if (CRC_Enable == true)
  {
    if (rec_cs != cal_cs)
    {
      stream.println("2213191634{\"status\":\"checksum failed\"}");
      return;
    }
  }

  // deserialize
  const uint8_t len = rawMessage.length();
  char json[len];
  rawMessage.toCharArray(json, len);
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    stream.println("858532069{\"status\":\"deserialize failed\"}");
    // stream.println(error.f_str());
    return;
  }

  const char *topic = doc["topic"];
  const String topic_name = String(topic);
  if (topic_name.compareTo("ros2_control") == 0)
  {
    velocity_lastcall_ms = millis();
    // OnVelocityControl = true;
    const double front_right_wheel_speed = doc["vel"][0]; // rad/s
    const double rear_right_wheel_speed = doc["vel"][1];  // rad/s
    const double rear_left_wheel_speed = doc["vel"][2];   // rad/s
    const double front_left_wheel_speed = doc["vel"][3];  // rad/s
    const uint32_t timeout = doc["timeout"];              // ms

    const double motor_1_velocity = front_right_wheel_speed * WHEEL_RADIUS;
    const double motor_2_velocity = rear_right_wheel_speed * WHEEL_RADIUS;
    const double motor_3_velocity = rear_left_wheel_speed * WHEEL_RADIUS;
    const double motor_4_velocity = front_left_wheel_speed * WHEEL_RADIUS;

    (timeout == 0) ? velocity_timeout_ms = velocity_lastcall_ms + DEFAULT_VEL_TIMEOUT : velocity_timeout_ms = velocity_lastcall_ms + timeout;
    Serial1.print("velocity_timeout_ms ");
    Serial1.println(velocity_timeout_ms);
    if (!Motor_Enable)
      enableMotor();

    motor1.setVelocity(motor_1_velocity);
    motor2.setVelocity(motor_2_velocity);
    motor3.setVelocity(motor_3_velocity);
    motor4.setVelocity(motor_4_velocity);
    stream.print(motor1.getSetSpeed());
    stream.print(" ");
    stream.print(motor2.getSetSpeed());
    stream.print(" ");
    stream.print(motor3.getSetSpeed());
    stream.print(" ");
    stream.print(motor4.getSetSpeed());
    stream.print("\n");
    // ros2 control response
    // doc["status"] = "ok";
    // char buffer[120];
    // serializeJson(doc, buffer);
    // String msg = String(buffer);
    // msg = crc_generate(msg) + msg + "\r\n";
    // stream.print("863713777{\"topic\":\"ros2_control\",\"status\":\"ok\"}\r\n");
    // stream.print("394310136{\"topic\":\"ros2_control\"}\r\n");
    stream.print("2110931352{\"status\":\"success\"}\r\n");
  }
  else if (topic_name.compareTo("ros2_state") == 0)
  {
    // // add battery value
    // // doc["bat"] = battery.getAverageVoltage();
    // doc["bat"] = trimDouble(vol);
    // doc["mAh"] = mA_sum * POWER_RAW_TO_MILI_WATT_HOUR;
    // doc["mWh"] = mW_sum * POWER_RAW_TO_MILI_WATT_HOUR;
    // // add velocity value
    // JsonArray velocity = doc.createNestedArray("vel");
    // velocity.add(motor1.getAverageSpeed());
    // velocity.add(motor2.getAverageSpeed());
    // velocity.add(motor3.getAverageSpeed());
    // velocity.add(motor4.getAverageSpeed());
    // // add position value
    // JsonArray position = doc.createNestedArray("pos");
    // position.add(motor1.getPosition());
    // position.add(motor2.getPosition());
    // position.add(motor3.getPosition());
    // position.add(motor4.getPosition());
    // // add imu value
    // if (IMU_AVAILABLE)
    // {
    //   JsonArray orientation = doc.createNestedArray("ori");
    //   orientation.add(trimDouble(ros2_sensor.orientation.x, 0));
    //   orientation.add(trimDouble(ros2_sensor.orientation.y, 0));
    //   orientation.add(trimDouble(ros2_sensor.orientation.z, 0));
    //   orientation.add(trimDouble(ros2_sensor.orientation.w, 0));

    //   JsonArray gyroscope = doc.createNestedArray("gyr");
    //   gyroscope.add(trimDouble(ros2_sensor.angular_velocity.x, 0));
    //   gyroscope.add(trimDouble(ros2_sensor.angular_velocity.y, 0));
    //   gyroscope.add(trimDouble(ros2_sensor.angular_velocity.z, 0));

    //   JsonArray accelerometer = doc.createNestedArray("acc");
    //   accelerometer.add(trimDouble(ros2_sensor.linear_acceleration.x, 0));
    //   accelerometer.add(trimDouble(ros2_sensor.linear_acceleration.y, 0));
    //   accelerometer.add(trimDouble(ros2_sensor.linear_acceleration.z, 0));

    //   JsonArray magnetic = doc.createNestedArray("mag");
    //   magnetic.add(trimDouble(ros2_sensor.magnetic_field.x, 0));
    //   magnetic.add(trimDouble(ros2_sensor.magnetic_field.y, 0));
    //   magnetic.add(trimDouble(ros2_sensor.magnetic_field.z, 0));
    // }
    // // doc["dur"] = micros() - start_t;

    // char buffer[500];
    // serializeJson(doc, buffer);
    // String msg = String(buffer);
    // msg = crc_generate(msg) + msg + "\r\n";
    // stream.print(msg);

    // char str[200];
    // sprintf(str, "{\"topic\":\"ros2_state\",\"bat\":%f,\"mAh\":%f,\"mWh\":%f,\"vel\":[0.00,0.00,0.00,0.00],\"pos\":[0.00,0.00,0.00,0.00]}",
    //         vol, mA_sum * POWER_RAW_TO_MILI_WATT_HOUR, mW_sum * POWER_RAW_TO_MILI_WATT_HOUR);

    String msg = "{\"vel\":[";
    msg += motor1.getAverageSpeed();
    msg += ",";
    msg += motor2.getAverageSpeed();
    msg += ",";
    msg += motor3.getAverageSpeed();
    msg += ",";
    msg += motor4.getAverageSpeed();
    msg += "],\"pos\":[";
    msg += motor1.getPosition();
    msg += ",";
    msg += motor2.getPosition();
    msg += ",";
    msg += motor3.getPosition();
    msg += ",";
    msg += motor4.getPosition();
    msg += "]";

    if (POWER_METTER_AVAILABLE)
    {
      msg += ",\"bat\":";
      msg += trimDouble(vol, 1);
      // msg += ",\"mA\":";
      // msg += trimDouble(mAmp, 2);
      // msg += ",\"mAh\":";
      // msg += trimDouble(mA_sum * POWER_RAW_TO_MILI_WATT_HOUR, 2);
      msg += ",\"Wh\":";
      msg += trimDouble(mW_sum * POWER_RAW_TO_WATT_HOUR, 2);
    }

    msg += "}";
    msg = crc_generate(msg) + msg + "\r\n";
    stream.print(msg);
  }
  else if (topic_name.compareTo("vel_control") == 0)
  {
    velocity_lastcall_ms = millis();
    // OnVelocityControl = true;
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

    (timeout == 0) ? velocity_timeout_ms = velocity_lastcall_ms + DEFAULT_VEL_TIMEOUT : velocity_timeout_ms = velocity_lastcall_ms + timeout;

    double motor_1_velocity = 0;
    double motor_2_velocity = 0;
    double motor_3_velocity = 0;
    double motor_4_velocity = 0;
    const double angular_tmp = angular_y * ANGULAR_VELOCITY_FACTOR;
    if (EN_MECANUM_WHEEL == true)
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

    motor1.setVelocity(motor_1_velocity);
    motor2.setVelocity(motor_2_velocity);
    motor3.setVelocity(motor_3_velocity);
    motor4.setVelocity(motor_4_velocity);
  }
  else if (topic_name.compareTo("pos_control") == 0)
  {
    //
  }
  else if (topic_name.compareTo("config") == 0)
  {
    // motor enable
    const String motorEnabled = doc["motor"];
    if (motorEnabled.compareTo("true") == 0)
    {
      enableMotor();
      stream.println("Success Enable Motor!");
    }
    else if (motorEnabled.compareTo("false") == 0)
    {
      disableMotor();
      stream.println("Success Disable Motor!");
    }

    // velocity log
    const String enableVelocityLog = doc["vel"];
    if (enableVelocityLog.compareTo("true") == 0)
    {
      EN_VELOCITY_LOG = true;
      stream.println("Success Enable Velocity Log!");
    }
    else if (enableVelocityLog.compareTo("false") == 0)
    {
      EN_VELOCITY_LOG = false;
      stream.println("Success Disable Velocity Log!");
    }

    // current log
    const String enableCurrentLog = doc["cur"];
    if (enableCurrentLog.compareTo("true") == 0)
    {
      EN_CURRENT_LOG = true;
      stream.println("Success Enable Current Log!");
    }
    if (enableCurrentLog.compareTo("false") == 0)
    {
      EN_CURRENT_LOG = false;
      stream.println("Success Disable Current Log!");
    }

    // imu log
    const String enableImuLog = doc["imu"];
    if (enableImuLog.compareTo("true") == 0)
    {
      EN_IMU_LOG = true;
      stream.println("Success Enable IMU Log!");
    }
    if (enableImuLog.compareTo("false") == 0)
    {
      EN_IMU_LOG = false;
      stream.println("Success Disable IMU Log!");
    }

    // crc
    const String enableCRC = doc["crc"];
    if (enableCRC.compareTo("true") == 0)
    {
      CRC_Enable = true;
      stream.println("Success Enable CRC!");
    }
    if (enableCRC.compareTo("false") == 0)
    {
      CRC_Enable = false;
      stream.println("Success Disable CRC!");
    }

    // pid
    const String pid = doc["pid"];
    if (pid.compareTo("vel") == 0)
    {
      const double p = doc["kp"];
      const double i = doc["ki"];
      const double d = doc["kd"];
      disableMotor();
      motor1.setVelocityPID(p, i, d);
      motor2.setVelocityPID(p, i, d);
      motor3.setVelocityPID(p, i, d);
      motor4.setVelocityPID(p, i, d);
      enableMotor();
      stream.println("Success config pid");
    }
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
    stream.println("reset successfully");
  }
  else if (topic_name.compareTo("info") == 0)
  {
    motor1.info(stream);
    motor2.info(stream);
    motor3.info(stream);
    motor4.info(stream);
  }
  // stream.println(micros() - start_t);
}

void logger()
{
  if (EN_VELOCITY_LOG)
  {
    Radio.print("MR");
    Radio.print(motor1.getSetSpeed());
    Radio.print(",ML");
    Radio.print(motor4.getSetSpeed());
    Radio.print(",M1=");
    Radio.print(motor1.getAverageSpeed());
    Radio.print(",M2=");
    Radio.print(motor2.getAverageSpeed());
    Radio.print(",M3=");
    Radio.print(motor3.getAverageSpeed());
    Radio.print(",M4=");
    Radio.print(motor4.getAverageSpeed());
    Radio.print("\r\n");
  }
  else if (EN_CURRENT_LOG)
  {
    Radio.print("INS_R=");
    Radio.print(currentSensor_1_2.getInstantCurrent());
    Radio.print(",INS_L=");
    Radio.print(currentSensor_3_4.getInstantCurrent());
    Radio.print(",AVE_R=");
    Radio.print(currentSensor_1_2.getAverageCurrent());
    Radio.print(",AVE_L=");
    Radio.print(currentSensor_3_4.getAverageCurrent());
    Radio.print(",FIL_R=");
    Radio.print(currentSensor_1_2.getAverageFilterCurrent());
    Radio.print(",FIL_L=");
    Radio.print(currentSensor_3_4.getAverageFilterCurrent());
    Radio.print("\r\n");
  }
  else if (EN_IMU_LOG && IMU_AVAILABLE)
  {
    Serial1.print("qx=");
    Serial1.print(ros2_sensor.orientation.x);
    Serial1.print(",qy=");
    Serial1.print(ros2_sensor.orientation.y);
    Serial1.print(",qz=");
    Serial1.print(ros2_sensor.orientation.z);
    Serial1.print(",qw=");
    Serial1.print(ros2_sensor.orientation.w);

    euler_t euler_out;
    quaternionToEuler(ros2_sensor.orientation.w, ros2_sensor.orientation.x, ros2_sensor.orientation.y, ros2_sensor.orientation.z, &euler_out, true);
    Serial1.print(",roll=");
    Serial1.print(euler_out.roll);
    Serial1.print(",pitch=");
    Serial1.print(euler_out.pitch);
    Serial1.print(",yaw=");
    Serial1.print(euler_out.yaw);

    Serial1.print(",gx=");
    Serial1.print(ros2_sensor.angular_velocity.x);
    Serial1.print(",gy=");
    Serial1.print(ros2_sensor.angular_velocity.y);
    Serial1.print(",gz=");
    Serial1.print(ros2_sensor.angular_velocity.z);

    Serial1.print(",ax=");
    Serial1.print(ros2_sensor.linear_acceleration.x);
    Serial1.print(",ay=");
    Serial1.print(ros2_sensor.linear_acceleration.y);
    Serial1.print(",az=");
    Serial1.print(ros2_sensor.linear_acceleration.z);

    Serial1.print(",mx=");
    Serial1.print(ros2_sensor.magnetic_field.x);
    Serial1.print(",my=");
    Serial1.print(ros2_sensor.magnetic_field.y);
    Serial1.print(",mz=");
    Serial1.print(ros2_sensor.magnetic_field.z);
    Serial1.println("");
  }
}