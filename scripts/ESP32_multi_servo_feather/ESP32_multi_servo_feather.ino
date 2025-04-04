#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 95   // Adjusted minimum pulse length (maps to -90 degrees)
#define SERVO_MAX 545  // Adjusted maximum pulse length (maps to +90 degrees)
#define NUM_SERVOS 16  // Number of servo channels

// On the Adafruit ESP32 Feather V2 the onboard LED might be on a different pin.
// Change this definition if your board uses a different pin.
#define LED_BUILTIN 13

// Array to track current servo positions
int currentPositions[NUM_SERVOS];

void setup() {
  Serial.begin(115200); // Use a higher baud rate for ESP32

  // Initialize I²C with the correct pins for the Feather V2.
  // For the ESP32-S3 on the Feather V2, the default I²C pins are SDA = 8 and SCL = 9.
  // Modify these if your wiring differs.
  Wire.begin(22, 20);

  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz

  // Initialize servo positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPositions[i] = 0; // Default position at 0 degrees
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
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

      if (channel >= 0 && channel < NUM_SERVOS) {
        int pulseWidth;

        if (position >= -90 && position <= 90) {
          pulseWidth = map(position, -90, 90, SERVO_MIN, SERVO_MAX);
        } else {  
          Serial.println("Invalid position value, must be between -90 and 90.");
          start = semicolonIndex + 1;
          continue;
        }

        pwm.setPWM(channel, 0, pulseWidth);

        // Update current position
        currentPositions[channel] = position;

        // Print current position and pulse width
        Serial.printf("Channel: %d, Position: %d degrees, Pulse Width: %d\n", channel, position, pulseWidth);
      } else {
        Serial.println("Invalid channel or out-of-range values.");
      }
    }
    start = semicolonIndex + 1;
  }
}
