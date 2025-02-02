#include <EnableInterrupt.h>
#include <AFMotor.h>

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

AF_DCMotor motor(2);
// Определяем пины для энкодеров
const int encoderPinLeft = A3; // Аналоговый пин A2 (цифровой пин 16)
const int encoderPinRight = A2; // Аналоговый пин A3 (цифровой пин 17)

void setup() {
    Serial.begin(9600);

    // Настройка пинов для энкодеров
    pinMode(encoderPinLeft, INPUT_PULLUP);
    pinMode(encoderPinRight, INPUT_PULLUP);

    // Включение программных прерываний для энкодеров
    enableInterrupt(encoderPinLeft, updateEncoderLeft, CHANGE);
    enableInterrupt(encoderPinRight, updateEncoderRight, CHANGE);

    motor.setSpeed(255);
    motor.run(RELEASE);
}

void loop() {
    // Для демонстрации выводим значения счетчиков энкодеров каждую секунду
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 200) {
        lastTime = millis();
        Serial.print("Left Encoder: ");
        Serial.print(encoderCountLeft);
        Serial.print(", Right Encoder: ");
        Serial.println(encoderCountRight);
    }
    motor.run(FORWARD);
}

void updateEncoderLeft() {
    encoderCountLeft++;
}

void updateEncoderRight() {
    encoderCountRight++;
}
