#include <Servo.h>

Servo leftServo;
Servo rightServo;

const int LEFT_SERVO_PIN = 9;
const int RIGHT_SERVO_PIN = 10;

int leftPosition = 90;
int rightPosition = 90;

void setup() {
  Serial.begin(9600);

  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);

  leftServo.write(90);
  rightServo.write(90);

  delay(500);
  Serial.println("Robot ready");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'L':
        leftPosition = 180;
        leftServo.write(leftPosition);
        Serial.println("Left arm up");
        break;

      case 'l':
        leftPosition = 90;
        leftServo.write(leftPosition);
        Serial.println("Left arm down");
        break;

      case 'R':
        rightPosition = 0;
        rightServo.write(rightPosition);
        Serial.println("Right arm up");
        break;

      case 'r':
        rightPosition = 90;
        rightServo.write(rightPosition);
        Serial.println("Right arm down");
        break;

      case 'C':
        leftPosition = 90;
        rightPosition = 90;

        leftServo.write(leftPosition);
        rightServo.write(rightPosition);

        Serial.println("Arms centered");
        break;

      case 'T':
        // Test both arms
        leftServo.write(0);
        rightServo.write(180);
        delay(1000);

        leftServo.write(90);
        rightServo.write(90);
        delay(1000);

        Serial.println("Movement test complete");
        break;
    }
  }
}
