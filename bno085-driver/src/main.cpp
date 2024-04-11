#include <Arduino.h>
#include <ArduinoJson.h>
#include "checksum.h"
// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS PB1
#define BNO08X_INT PB0

// For SPI mode, we also need a RESET
// #define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET PA4

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
JsonDocument doc;

uint64_t last_t = micros();
String messageFromBridge = "";
bool CRC_Enable = false;

struct vector4Double
{
  double x;
  double y;
  double z;
  double w;
};
struct vector3Double
{
  double x;
  double y;
  double z;
};
struct euler_t
{
  double roll, pitch, yaw;
};
struct sensors
{
  vector4Double orientation;
  vector3Double angular_velocity;
  vector3Double linear_acceleration;
  vector3Double magnetic_field;
  double temperature;
} ros2_sensor;

void setReports();
void msgProcess(String, Stream &);
double trimDouble(double, uint8_t);
void tick();

HardwareTimer timer(TIM1);
uint64_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 10 == 0)
  {
    tick();
  }
  // if (timer_count % 100 == 0)
  // {
  //   Serial.print(ros2_sensor.orientation.x);
  //   Serial.print(" ");
  //   Serial.print(ros2_sensor.orientation.y);
  //   Serial.print(" ");
  //   Serial.print(ros2_sensor.orientation.z);
  //   Serial.print(" ");
  //   Serial.print(ros2_sensor.orientation.w);
  //   Serial.println();
  // }
}

void setup(void)
{
  Serial.begin(460800);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  // if (!bno08x.begin_I2C())
  // {
  // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
  // UART buffer!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT))
  {
    Serial.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++)
  {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();
  timer.setPrescaleFactor(9999);
  timer.setOverflow(9);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh();
  timer.resume();

  Serial.println("Reading events");
}

void loop()
{
}
void tick()
{
  if (bno08x.wasReset())
  {
    Serial.print("sensor was reset ");
    setReports();
  }

  uint64_t start_t = micros();
  if (!bno08x.getSensorEvent(&sensorValue))
  {
    return;
  }
  switch (sensorValue.sensorId)
  {
  case SH2_ARVR_STABILIZED_GRV:
    ros2_sensor.orientation.x = sensorValue.un.arvrStabilizedGRV.i;
    ros2_sensor.orientation.y = sensorValue.un.arvrStabilizedGRV.j;
    ros2_sensor.orientation.z = sensorValue.un.arvrStabilizedGRV.k;
    ros2_sensor.orientation.w = sensorValue.un.arvrStabilizedGRV.real;
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    ros2_sensor.angular_velocity.x = sensorValue.un.gyroscope.x;
    ros2_sensor.angular_velocity.y = sensorValue.un.gyroscope.y;
    ros2_sensor.angular_velocity.z = sensorValue.un.gyroscope.z;
    break;
  case SH2_LINEAR_ACCELERATION:
    ros2_sensor.linear_acceleration.x = sensorValue.un.linearAcceleration.x;
    ros2_sensor.linear_acceleration.y = sensorValue.un.linearAcceleration.y;
    ros2_sensor.linear_acceleration.z = sensorValue.un.linearAcceleration.z;
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    ros2_sensor.magnetic_field.x = sensorValue.un.magneticField.x;
    ros2_sensor.magnetic_field.y = sensorValue.un.magneticField.y;
    ros2_sensor.magnetic_field.z = sensorValue.un.magneticField.z;
    break;
  }
  // uint64_t now_t = micros();
  // Serial.print(start_t - last_t);
  // Serial.print(" ");
  // Serial.println(now_t - start_t);
  // last_t = start_t;
}

void serialEvent1()
{
  // messageFromBridge = Serial1.readStringUntil('\n');
  // msgProcess(messageFromBridge, Serial1);
  // messageFromBridge = "";
  while (Serial1.available())
  {
    char tempChar = (char)Serial1.read();
    if (tempChar != '\n')
    {
      messageFromBridge += tempChar;
    }
    else
    {
      msgProcess(messageFromBridge, Serial1);
      messageFromBridge = "";
    }
  }
}

// Here is where you define the sensor outputs you want to receive
void setReports()
{
  Serial.println("Setting bno085 reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_GRV))
  {
    Serial.println("Could not enable stabilized remote vector");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
  {
    Serial.println("Could not enable gyroscope calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
  {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED))
  {
    Serial.println("Could not enable magnetic field calibrated");
  }
}

void msgProcess(String lightCmd, Stream &stream)
{
  uint64_t start_t = micros();
  uint32_t idx = lightCmd.indexOf('{'); //

  String cmd_cs = lightCmd.substring(0, idx); // received checksum
  uint32_t rec_cs = 0;
  for (unsigned int i = 0; i < cmd_cs.length(); i++)
  {
    rec_cs = rec_cs * 10 + (cmd_cs[i] - '0');
  }
  lightCmd = lightCmd.substring(idx);       // split light comment from message
  uint32_t cal_cs = crc_generate(lightCmd); // calculated checksum

  // checksum
  if (CRC_Enable == true)
  {
    if (rec_cs != cal_cs)
      return;
  }
  const uint8_t len = lightCmd.length();
  char json[len];
  lightCmd.toCharArray(json, len);
  // Serial1.print(lightCmd);

  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    stream.print("deserializeJson() failed, error: ");
    stream.print(lightCmd);
    stream.print("\t");
    stream.println(error.f_str());
    return;
  }
  const char *topic = doc["topic"];
  const String topic_name = String(topic);
  if (topic_name.compareTo("ros2_state") == 0)
  {
    // JsonArray orientation = doc["qua"].to<JsonArray>();
    // orientation.add(trimDouble(ros2_sensor.orientation.x, 0));
    // orientation.add(trimDouble(ros2_sensor.orientation.y, 0));
    // orientation.add(trimDouble(ros2_sensor.orientation.z, 0));
    // orientation.add(trimDouble(ros2_sensor.orientation.w, 0));

    // JsonArray gyroscope = doc["gyr"].to<JsonArray>();
    // gyroscope.add(trimDouble(ros2_sensor.angular_velocity.x, 0));
    // gyroscope.add(trimDouble(ros2_sensor.angular_velocity.y, 0));
    // gyroscope.add(trimDouble(ros2_sensor.angular_velocity.z, 0));

    // JsonArray accelerometer = doc["acc"].to<JsonArray>();
    // accelerometer.add(trimDouble(ros2_sensor.linear_acceleration.x, 0));
    // accelerometer.add(trimDouble(ros2_sensor.linear_acceleration.y, 0));
    // accelerometer.add(trimDouble(ros2_sensor.linear_acceleration.z, 0));

    // JsonArray magnetic = doc["mag"].to<JsonArray>();
    // magnetic.add(trimDouble(ros2_sensor.magnetic_field.x, 0));
    // magnetic.add(trimDouble(ros2_sensor.magnetic_field.y, 0));
    // magnetic.add(trimDouble(ros2_sensor.magnetic_field.z, 0));

    // char buffer[500];
    // serializeJson(doc, buffer);
    // String msg = String(buffer);
    // msg = crc_generate(msg) + msg + "\r\n";

    String msg = "{\"qua\":[";
    msg += ros2_sensor.orientation.x;
    msg += ",";
    msg += ros2_sensor.orientation.y;
    msg += ",";
    msg += ros2_sensor.orientation.z;
    msg += ",";
    msg += ros2_sensor.orientation.w;

    msg += "],\"gyr\":[";
    msg += ros2_sensor.angular_velocity.x;
    msg += ",";
    msg += ros2_sensor.angular_velocity.y;
    msg += ",";
    msg += ros2_sensor.angular_velocity.z;

    msg += "],\"acc\":[";
    msg += ros2_sensor.linear_acceleration.x;
    msg += ",";
    msg += ros2_sensor.linear_acceleration.y;
    msg += ",";
    msg += ros2_sensor.linear_acceleration.z;

    msg += "],\"mag\":[";
    msg += ros2_sensor.magnetic_field.x;
    msg += ",";
    msg += ros2_sensor.magnetic_field.y;
    msg += ",";
    msg += ros2_sensor.magnetic_field.z;

    msg += "]}";
    msg = crc_generate(msg) + msg + "\r\n";
    stream.print(msg);
  }
}

double trimDouble(double in, uint8_t num = 2)
{
  if (num == 0)
    return in;
  else
  {
    const uint16_t sc = 10 ^ num;
    return round(in * sc) / sc;
  }
}
