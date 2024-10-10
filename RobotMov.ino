#include <AccelStepper.h>

// Initialize the stepper library
AccelStepper stepper1(4, 4, 5, 6, 7);
AccelStepper stepper2(4, 8, 9, 10, 11);

void setup() {
  Serial.setTimeout(0);

  // Define maximum speed and acceleration for the motors
  stepper1.setMaxSpeed(38);  
  stepper1.setAcceleration(150); 
  stepper2.setMaxSpeed(38); 
  stepper2.setAcceleration(150); 

  Serial.begin(2000000);  // Initialize the Serial Port
}

void loop() {

  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); // Read the message until a newline character is found

    // Check if the message is in the correct format
    int commaIndex = message.indexOf(',');
    if (commaIndex != -1) {
      // Extract the two parts of the message
      String value1_str = message.substring(0, commaIndex);
      String value2_str = message.substring(commaIndex + 1);

      // Convert string to integers
      int value1 = value1_str.toInt();
      int value2 = value2_str.toInt();

      // Desired positions
      stepper1.moveTo(value1);
      stepper2.moveTo(value2);

    } else {
      Serial.println("Error: Incorrect message format");
    }
  }

  // Move the motors to the desired position simultaneously
  stepper1.run();
  stepper2.run();
}

