#include <Arduino.h>
#include <PID_v1.h>

// define motor 1
#define MOTOR_1_A PB3
#define MOTOR_1_B PA15
#define MOTOR_1_PWM PB9
#define MOTOR_1_DIR PB8
// define motor 2
#define MOTOR_2_A PA12
#define MOTOR_2_B PA11
#define MOTOR_2_PWM PB5
#define MOTOR_2_DIR PB4
// define motor 3
#define MOTOR_3_A PC15
#define MOTOR_3_B PA0
#define MOTOR_3_PWM PB0
#define MOTOR_3_DIR PB1
// define motor 4
#define MOTOR_4_A PC13
#define MOTOR_4_B PC14
#define MOTOR_4_PWM PA4
#define MOTOR_4_DIR PA5

// define sensor
#define DIS_SEN PA1

HardwareSerial Serial2(PA3, PA2);

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

void velocityGenerate(double input)
{
  //
}

// -------------------------------------------------ENCODER HANDLE--------------------------------------------------------
int drive(int);
int stop();
class EncoderHandle
{
private:
  const float ppr = 2970.0;
  uint32_t a;
  uint32_t b;
  uint32_t p;
  float v;
  uint32_t last_t;
  uint32_t last_p;

  uint32_t tick()
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
  EncoderHandle()
  {
    this->a = 0;
    this->b = 0;
    this->p = 0;
    this->last_t = millis();
    this->last_p = 0;
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
} encoder1, encoder2, encoder3, encoder4;

void EncoderHandle_1_A()
{
  encoder1.channel_a();
}
void EncoderHandle_1_B()
{
  encoder1.channel_b();
}
void EncoderHandle_2_A()
{
  encoder2.channel_a();
}
void EncoderHandle_2_B()
{
  encoder2.channel_b();
}
void EncoderHandle_3_A()
{
  encoder3.channel_a();
}
void EncoderHandle_3_B()
{
  encoder3.channel_b();
}
void EncoderHandle_4_A()
{
  encoder4.channel_a();
}
void EncoderHandle_4_B()
{
  encoder4.channel_b();
}

uint32_t last_t = millis();
uint32_t last_p = encoder1.pulses();
uint32_t tick_t = millis();

void setup()
{
  // Start serial 1 as external control communication
  Serial1.begin(115200);
  // Start serial 2 as rf communication
  Serial2.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_1_A, INPUT_PULLUP);
  pinMode(MOTOR_1_B, INPUT_PULLUP);
  pinMode(MOTOR_2_A, INPUT_PULLUP);
  pinMode(MOTOR_2_B, INPUT_PULLUP);
  pinMode(MOTOR_3_A, INPUT_PULLUP);
  pinMode(MOTOR_3_B, INPUT_PULLUP);
  pinMode(MOTOR_4_A, INPUT_PULLUP);
  pinMode(MOTOR_4_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_A), EncoderHandle_1_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_B), EncoderHandle_1_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_A), EncoderHandle_2_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_B), EncoderHandle_2_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_A), EncoderHandle_3_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_B), EncoderHandle_3_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_A), EncoderHandle_4_A, HIGH);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_B), EncoderHandle_4_B, HIGH);

  pinMode(DIS_SEN, INPUT);

  // xTaskCreate(TaskLed, // Task function
  //             "Led",   // Task name
  //             128,     // Stack size
  //             NULL,
  //             0, // Priority
  //             NULL);

  // /**
  //  * Create a binary semaphore.
  //  * https://www.freertos.org/xSemaphoreCreateBinary.html
  //  */
  // interruptSemaphore = xSemaphoreCreateBinary();
  // if (interruptSemaphore != NULL)
  // {
  //   // Attach interrupt for Arduino digital pin
  //   attachInterrupt(digitalPinToInterrupt(MOTOR_1_A), interruptHandler, LOW);
  // }

  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  pinMode(MOTOR_3_PWM, OUTPUT);
  pinMode(MOTOR_4_PWM, OUTPUT);
  pinMode(MOTOR_1_DIR, OUTPUT);
  pinMode(MOTOR_2_DIR, OUTPUT);
  pinMode(MOTOR_3_DIR, OUTPUT);
  pinMode(MOTOR_4_DIR, OUTPUT);
}

void loop()
{
  // const uint32_t d_t = millis() - last_t;
  // if (d_t >= 1000)
  // {
  //   const uint32_t d_p = encoder1.pulses() - last_p;
  //   float v = (d_p * 60 * 1000) / (d_t * 2970.0);
  //   last_t = millis();
  //   last_p = encoder1.pulses();
  //   Serial1.print(last_p);
  //   Serial1.print(':');
  //   Serial1.print(v);
  //   Serial1.println('.');
  // }
  if (millis() - tick_t > 500)
  {
    Serial1.print(encoder1.velocity());
    Serial1.print(':');
    Serial1.print(encoder2.velocity());
    Serial1.print(':');
    Serial1.print(encoder3.velocity());
    Serial1.print(':');
    Serial1.print(encoder4.velocity());
    Serial1.println('.');
    tick_t = millis();
  }
}

int drive(int speed)
{
  return 1;
}

int stop()
{
  return 1;
}