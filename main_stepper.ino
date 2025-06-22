const int STEP_DELAY_US = 900;       // microseconds delay between step HIGH/LOW
const int STEPS_PER_REV = 3200;      // 200 * 16 microstepping
const float DEG_PER_INPUT_UNIT = 1;  // 18.6;
const float STEPS_PER_INPUT_UNIT = (STEPS_PER_REV / 360.0) * DEG_PER_INPUT_UNIT;
const int MIN_STEP_THRESHOLD = 2;

/* -------------------------- */

const int dirPin_y = 2;
const int stepPin_y = 3;
const int enPin_y = 6;

const int dirPin_x = 4;
const int stepPin_x = 5;
const int enPin_x = 7;

/* -------------------------- */

String packet;
String lastCommand;

void setup()
{
    Serial.begin(115200);

    pinMode(dirPin_x, OUTPUT);
    pinMode(stepPin_x, OUTPUT);
    pinMode(enPin_x, OUTPUT);

    pinMode(dirPin_y, OUTPUT);
    pinMode(stepPin_y, OUTPUT);
    pinMode(enPin_y, OUTPUT);

    // Enable motors at startup
    digitalWrite(enPin_x, LOW);  // LOW = enabled
    digitalWrite(enPin_y, LOW);
}

void loop()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            lastCommand = packet;
            packet = "";
        }
        else {
            packet += c;
        }
    }

    if (lastCommand.length()) {
        handle(lastCommand);
        lastCommand = "";
    }
}

void handle(const String& cmd)
{
    int commaIndex = cmd.indexOf(',');
    if (commaIndex == -1 || commaIndex == cmd.length() - 1) return;

    float horizInput = cmd.substring(0, commaIndex).toFloat();
    float vertInput = cmd.substring(commaIndex + 1).toFloat();

    float horizStepsF = horizInput * STEPS_PER_INPUT_UNIT;
    float vertStepsF = vertInput * STEPS_PER_INPUT_UNIT;

    int horizSteps = abs(round(horizStepsF));
    int vertSteps = abs(round(vertStepsF));

    bool horizDir = (horizStepsF >= 0);
    bool vertDir = !(vertStepsF >= 0);

    // If no meaningful movement, disable motors and return
    if (horizSteps < MIN_STEP_THRESHOLD && vertSteps < MIN_STEP_THRESHOLD) {
        digitalWrite(enPin_x, HIGH);  // HIGH = disable
        digitalWrite(enPin_y, HIGH);
        return;
    }

    // Enable before stepping
    digitalWrite(enPin_x, LOW);
    digitalWrite(enPin_y, LOW);

    // Perform movement
    if (horizSteps >= MIN_STEP_THRESHOLD)
        moveStepper(dirPin_x, stepPin_x, horizDir, horizSteps);

    if (vertSteps >= MIN_STEP_THRESHOLD)
        moveStepper(dirPin_y, stepPin_y, vertDir, vertSteps);

    // Disable motors to prevent noise and heating
    digitalWrite(enPin_x, HIGH);
    digitalWrite(enPin_y, HIGH);
}

void moveStepper(int dirPin, int stepPin, bool direction, int steps)
{
    digitalWrite(dirPin, direction ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(STEP_DELAY_US);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(STEP_DELAY_US);
    }
}
