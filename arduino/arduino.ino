#include <Servo.h>
#include <stdlib.h>

short base_goto, shoulder_goto, elbow_goto, claw_goto;
Servo base, shoulder, elbow, claw;
const int BASE_PIN = 8; // temporary
const int SHOULDER_PIN = 9; // temporary
const int ELBOW_PIN = 10; // temporary
const int CLAW_PIN = 11; // temporary
const int SERIAL_TIMEOUT = 50;

typedef struct Command {
    short base;
    short shoulder;
    short elbow;
    short claw;
} Command;

const short MAX = 2400; //2400
const short MIN = 250; //250

int parse_size = 0;
byte *byte_buffer;

Command message = {
    MAX, MAX, MAX, MAX 
};

void setup() {
    parse_size = 4 * sizeof(short);
    byte_buffer = malloc(parse_size);

    base.attach(BASE_PIN);
    shoulder.attach(SHOULDER_PIN);
    elbow.attach(ELBOW_PIN);
    claw.attach(CLAW_PIN);

    base.writeMicroseconds(message.base);
    shoulder.writeMicroseconds(message.shoulder);
    elbow.writeMicroseconds(message.elbow);
    claw.writeMicroseconds(message.claw);

    Serial.begin(115200);
    Serial.setTimeout(50);
    Serial.print("\rSerial began\n");

    Serial.print("base: ");
    Serial.print(message.base);
    Serial.print(", shoulder: ");
    Serial.print(message.shoulder);
    Serial.print(", elbow: ");
    Serial.print(message.elbow);
    Serial.print(", claw: ");
    Serial.print(message.claw);
    Serial.print('\n');
}

void loop() {

}

void serialEvent() {
    if (Serial.read() != '\r') return;
    Serial.readBytesUntil('\n', (char*)&message, 8);

    base.writeMicroseconds(message.base);
    shoulder.writeMicroseconds(message.shoulder);
    elbow.writeMicroseconds(message.elbow);
    claw.writeMicroseconds(message.claw);

    if (message.base == 0xFFFFFFFF) return;
    if (message.shoulder == 0xFFFFFFFF) return;
    if (message.elbow == 0xFFFFFFFF) return;
    if (message.base == 0xFFFFFFFF) return;

    base.attach(3);
    shoulder.attach(3);
    elbow.attach(3);
    claw.attach(3);

    // Serial.print("base: ");
    // Serial.print(message.base);
    // Serial.print(", shoulder: ");
    // Serial.print(message.shoulder);
    // Serial.print(", elbow: ");
    // Serial.print(message.elbow);
    // Serial.print(", claw: ");
    // Serial.print(message.claw);
    // Serial.print('\n');
}
