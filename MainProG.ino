#include <EnableInterrupt.h>
#include <AFMotor.h>

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

AF_DCMotor MotorLeft(2);
AF_DCMotor MotorRight(3);

const int encoderPinLeft = A3;
const int encoderPinRight = A2;

// Константы для PID
double Kp = 2.0;   // Пропорциональная составляющая
double Ki = 0.8;   // Интегральная составляющая
double Kd = 0.05;  // Дифференциальная составляющая

// Переменные для хранения текущего состояния
double setPointLeft = 150.0;
double setPointRight = 100.0;
double currentSpeedLeft = 0.0;
double currentSpeedRight = 0.0;

// Переменные для PID-регулирования
double prevErrorLeft = 0.0;
double prevErrorRight = 0.0;
double integralLeft = 0.0;
double integralRight = 0.0;

// Частота обновления PID (в Гц)
int updateRate = 50;  // Обновление каждые 50 мс
unsigned long previousMillis = 0;

void calculatePID(double &output, double currentSpeed, double setPoint, double &prevError, double &integral) {
  double error = setPoint - currentSpeed;
  integral += error * (1.0 / updateRate);
  double derivative = (error - prevError) * updateRate;
  output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
}

void setMotorSpeed(AF_DCMotor *motor, int speed) {
  bool direction;
  if (speed < 0) {
    speed = -speed;
    direction = false;
  } else {
    direction = true;
  }
  if (speed > 255) speed = 255;

  motor->setSpeed(speed);

  if (direction) {
    motor->run(BACKWARD);  // Направление вперед
  } else {
    motor->run(FORWARD);  // Направление назад
  }
}

void updateEncoderLeft() {
    encoderCountLeft++;
}

void updateEncoderRight() {
    encoderCountRight++;
}

double calculateCurrentSpeed(volatile long &encoderCount, double timeInterval) {
    double currentSpeed = (encoderCount * 1.0 / timeInterval) * 60.0; // Преобразование в обороты в минуту
    encoderCount = 0; // Сброс счетчика после расчета
    Serial.print("Left Speed: ");
    Serial.println(currentSpeed);
    return currentSpeed;
}

void setup() {
  Serial.begin(9600);
  // Настройка пинов для энкодеров
  pinMode(encoderPinLeft, INPUT_PULLUP);
  pinMode(encoderPinRight, INPUT_PULLUP);

  // Включение программных прерываний для энкодеров
  enableInterrupt(encoderPinLeft, updateEncoderLeft, CHANGE);
  enableInterrupt(encoderPinRight, updateEncoderRight, CHANGE);
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= (1000 / updateRate)) {
        previousMillis = currentMillis;

        // Расчет текущих скоростей колес
        currentSpeedLeft = calculateCurrentSpeed(encoderCountLeft, 1.0 / updateRate);
        //currentSpeedRight = calculateCurrentSpeed(encoderCountRight, 1.0 / updateRate);

        // Вычисление выхода PID для левого и правого колес
        double outputLeft, outputRight;
        calculatePID(outputLeft, currentSpeedLeft, setPointLeft, prevErrorLeft, integralLeft);
        //calculatePID(outputRight, currentSpeedRight, setPointRight, prevErrorRight, integralRight);

        // Установка скорости двигателей
        setMotorSpeed(&MotorLeft, outputLeft);
        //setMotorSpeed(MotorRight, outputRight);

        // Вывод данных для отладки
        
        //Serial.print(" Right Speed: ");
        //Serial.println(currentSpeedRight);
    }
}
