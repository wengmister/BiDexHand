#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 95   // Adjusted minimum pulse length (maps to -90 degrees)
#define SERVO_MAX 510  // Adjusted maximum pulse length (maps to +90 degrees)
#define NUM_SERVOS 16  // Number of servo channels

// Array to track current servo positions
int currentPositions[NUM_SERVOS];

void setup() {
  Serial.begin(115200); // Use a higher baud rate for ESP32
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz

  // Initialize servo positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPositions[i] = 0; // Default position at 0 degrees
  }
}
 
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read incoming command
    handleCommands(command);
  }
}

void handleCommands(String command) {
  // Command format: "ch1,pos1;ch2,pos2;..." (e.g., "13,45;14,-30")
  int start = 0;
  while (start < command.length()) {
    int semicolonIndex = command.indexOf(';', start);
    if (semicolonIndex == -1) semicolonIndex = command.length();

    String subCommand = command.substring(start, semicolonIndex);
    int commaIndex = subCommand.indexOf(',');
    if (commaIndex > 0) {
      int channel = subCommand.substring(0, commaIndex).toInt();
      int position = subCommand.substring(commaIndex + 1).toInt();

      if (channel >= 0 && channel < NUM_SERVOS && position >= -90 && position <= 90) {
        int pulseWidth = map(position, -90, 90, SERVO_MIN, SERVO_MAX);
        pwm.setPWM(channel, 0, pulseWidth);

        // Update current position
        currentPositions[channel] = position;

        // Print current position and pulse width
        // Serial.println("Channel: %d, Position: %d degrees, Pulse Width: %d\n", channel, position, pulseWidth);
      } else {
        Serial.println("Invalid command or out-of-range values.");
      }
    }
    start = semicolonIndex + 1;
  }
}
