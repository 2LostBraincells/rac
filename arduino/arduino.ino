#include <Servo.h>
#include <stdlib.h>

Servo shoulder, elbow;
const int SHOULDER_PIN = 5; // temporary
const int ELBOW_PIN = 4; // temporary
const int SERIAL_TIMEOUT = 50;

int parse_size = 0;
byte *byte_buffer;

void setup() {
  parse_size = 4 * sizeof(short);
  byte_buffer = malloc(parse_size);

  shoulder.attach(SHOULDER_PIN);
  shoulder.attach(ELBOW_PIN);

  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("Serial began");
}

void loop() {
  if (Serial.available()) {
   Serial.readBytes(byte_buffer, parse_size);
  }

  for (int i = 0; i < parse_size; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(*byte_buffer);
    Serial.print(", ");
  }
  Serial.println();
}
