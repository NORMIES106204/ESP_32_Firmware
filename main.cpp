#include <Arduino.h>
#include <QTRSensors.h>
#include <Wifi.h>




// === QTR Sensor Configuration ===
const uint8_t sensorPins[6] = {36, 39, 34, 35, 32, 33};
const uint8_t NUM_SENSORS = 6;
const int sensorEnablePin = 4;
const int threshold = 4090; // Logic threshold

QTRSensors qtr;

// === Motor Pins ===
const int ENB = 25;
const int IN3 = 27;
const int IN4 = 14;
const int ENA = 26;
const int IN1 = 13;
const int IN2 = 12;

// === PWM channels (ESP32 uses ledcWrite)
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmFreq = 1000;
const int pwmRes = 8;

// === Motor and PID Settings ===
const int baseSpeed = 70;
const int maxSpeed = 200;

float Kp = 15.0, Ki = 0.0, Kd = 15.0;

float error = 0, lastError = 0, integral = 0;

int logicValues[NUM_SENSORS];
int weights[6] = {-3, -2, -1, 1, 2, 3}; // Weight for PID from S0 to S5

void setMotor(int leftSpeed, int rightSpeed)
{
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(pwmChannelA, leftSpeed);
  ledcWrite(pwmChannelB, rightSpeed);
}

void stopMotors()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
}

void setup()
{
  Serial.begin(9600);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Sensor enable
  pinMode(sensorEnablePin, OUTPUT);
  digitalWrite(sensorEnablePin, HIGH);

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(pwmChannelA, pwmFreq, pwmRes);
  ledcAttachPin(ENA, pwmChannelA);
  ledcSetup(pwmChannelB, pwmFreq, pwmRes);
  ledcAttachPin(ENB, pwmChannelB);

  // QTR sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);

  Serial.println("Calibrating...");
  for (int i = 0; i < 200; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    qtr.calibrate();
    digitalWrite(LED_BUILTIN, LOW);
    delay(20);
  }
  Serial.println("Calibration done.");
}

void loop()
{
  uint16_t sensorValues[NUM_SENSORS];
  qtr.read(sensorValues);

  int weightedSum = 0;
  int activeCount = 0;

  // === Print RAW ===
  Serial.print("RAW:    ");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensorValues[i]);
    Serial.print("  ");
  }
  Serial.println();

  // === Convert to logic and apply weights ===
  Serial.print("LOGIC:  ");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    logicValues[i] = sensorValues[i] > threshold ? 1 : 0; // 1 = line detected (dark)
    Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(logicValues[i]);
    Serial.print("  ");

    if (logicValues[i])
    {
      weightedSum += logicValues[i] * weights[i];
      activeCount++;
    }
  }
  Serial.println();

  // === PID Calculation using logic-weighted average ===
  if (activeCount > 0)
  {
    float logicPos = (float)weightedSum / activeCount;
    error = logicPos;
    integral += error;
    float derivative = error - lastError;
    lastError = error;

    float correction = Kp * error + Ki * integral + Kd * derivative;
    correction = constrain(correction, -maxSpeed, maxSpeed);

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    setMotor(leftSpeed, rightSpeed);
  }
  else
  {
    stopMotors();
    Serial.println("■ STOP - No line detected");
  }

  Serial.println();
}
