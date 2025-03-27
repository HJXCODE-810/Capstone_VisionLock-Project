#include <AccelStepper.h>

// Motor connections
#define MOTOR1_STEP 2
#define MOTOR1_DIR 3
#define MOTOR2_STEP 4
#define MOTOR2_DIR 5
#define ENABLE_PIN 8

AccelStepper stepperX(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);

void setup() {
  Serial.begin(9600);
  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable stepper drivers
  
  // Horizontal motor (X-axis)
  stepperX.setMaxSpeed(1000);
  stepperX.setAcceleration(500);
  
  // Vertical motor (Y-axis)
  stepperY.setMaxSpeed(800);
  stepperY.setAcceleration(300);
  
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    if (commaIndex != -1) {
      int x_steps = data.substring(0, commaIndex).toInt();
      int y_steps = data.substring(commaIndex + 1).toInt();
      
      // Move motors to new relative positions
      stepperX.move(x_steps);
      stepperY.move(y_steps);
      
      // Run both steppers simultaneously
      while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
        stepperX.run();
        stepperY.run();
      }
    }
  }
}
