#include "API.h"


// ── MPU angle tracking ────────────────────────────────────────────────────
// Improvement #2: prevAngle accumulation removed. Each turn now captures its
// own startAngle and targets startAngle ± 85°, so drift never compounds across turns.
float angle = 0.0, rawAngle = 0.0;


// ── Encoder state ─────────────────────────────────────────────────────────
const int readingsBeforeClearing = 100;
int readingsCount = 0;

float distancePerPulse = 0.1;
volatile long encoderCountRight = 0, encoderCountLeft = 0;
float distanceLeft = 0.0, distanceRight = 0.0;


// ── Motor primitives ──────────────────────────────────────────────────────
void Forward(void) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void Stop(void) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
void Right(void) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void Left(void) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}


// ── Wall sensors ──────────────────────────────────────────────────────────
bool wallFront() { return getDistance(sonar[S_MIDDLE]) < (SENSOR_LIMIT - 3); }
bool wallRight()  { return getDistance(sonar[S_RIGHT])  < SENSOR_LIMIT; }
bool wallLeft()   { return getDistance(sonar[S_LEFT])   < SENSOR_LIMIT; }


// ── Forward movement ──────────────────────────────────────────────────────
void moveForward() {
    encoderCountLeft  = 0;
    encoderCountRight = 0;
    distanceLeft  = 0.0;
    distanceRight = 0.0;

    while (distanceLeft < 9 || distanceRight < 9) {
        Forward();
        distanceLeft  = encoderCountLeft  * distancePerPulse;
        distanceRight = encoderCountRight * distancePerPulse;
    }
    Stop();
}


// ── Turn control ──────────────────────────────────────────────────────────
const float correctionFactor = 2.3;
const int   baseSpeed        = 220;

/**
 * Improvement #2: delta-angle turns.
 *
 * Old approach: targetAngle = prevAngle + 85, where prevAngle accumulated across
 * all previous turns. Calling mpu.calcOffsets() each time (now removed) reset the
 * gyro baseline, making prevAngle meaningless and causing each successive turn to
 * overshoot by an increasing margin.
 *
 * New approach: snapshot startAngle at entry, target startAngle ± 85°.
 * Drift never compounds. No prevAngle needed.
 *
 * Sign convention (MPU6050_light, Z-axis up):
 *   Right (CW from above) → getAngleZ() increases
 *   Left  (CCW from above) → getAngleZ() decreases
 * If your MPU is mounted inverted, swap the +/- and the while conditions.
 */
void turnRight() {
    mpu.update();
    float startAngle  = mpu.getAngleZ();
    float targetAngle = startAngle + 85.0;

    while (mpu.getAngleZ() < targetAngle) {
        mpu.update();
        float correction = (targetAngle - mpu.getAngleZ()) * correctionFactor;
        Right();
        analogWrite(ENA, max((int)(baseSpeed + correction), 100));
        analogWrite(ENB, max((int)(baseSpeed - correction), 100));
    }
    Stop();
}

void turnLeft() {
    mpu.update();
    float startAngle  = mpu.getAngleZ();
    float targetAngle = startAngle - 85.0;

    while (mpu.getAngleZ() > targetAngle) {
        mpu.update();
        float correction = (mpu.getAngleZ() - targetAngle) * correctionFactor;
        Left();
        analogWrite(ENA, max((int)(baseSpeed + correction), 100));
        analogWrite(ENB, max((int)(baseSpeed - correction), 100));
    }
    Stop();
}


// ── Orientation / coordinate helpers ─────────────────────────────────────
int orientation(int orient, char turning) {
    if (turning == 'L') {
        orient--;
        if (orient == -1) orient = 3;
    } else if (turning == 'R') {
        orient++;
        if (orient == 4) orient = 0;
    } else if (turning == 'B') {
        orient = (orient + 2) % 4;
    }
    return orient;
}

void updateCoordinates(int orient, int *new_x, int *new_y) {
    if      (orient == FORWARD)  (*new_y)++;
    else if (orient == RIGHT)    (*new_x)++;
    else if (orient == BACKWARD) (*new_y)--;
    else if (orient == LEFT)     (*new_x)--;
}


// ── Encoder ISRs ──────────────────────────────────────────────────────────
void EncoderLeftHandler() {
    int stateA = digitalRead(encoderLeft_A);
    int stateB = digitalRead(encoderLeft_B);
    encoderCountLeft += (stateA == stateB) ? 1 : -1;
}
void EncoderRightHandler() {
    int stateA = digitalRead(encoderRight_A);
    int stateB = digitalRead(encoderRight_B);
    encoderCountRight += (stateA == stateB) ? 1 : -1;
}


// ── Distance measurement ──────────────────────────────────────────────────
/**
 * Improvement #4: AVG_NUM reduced 5→3, delay reduced 30ms→20ms.
 * Old: 5 pings × 30ms × 3 sensors = 450ms blocked per loop.
 * New: 3 pings × 20ms × 3 sensors = 180ms — 60% faster, negligible accuracy loss
 * (maze walls don't move between readings).
 */
uint32_t getDistance(NewPing sonar) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < AVG_NUM; i++) {
        sum += sonar.ping_cm();
        delay(20);   // was 30ms
    }
    return sum / AVG_NUM;
}


// ── MPU FIFO reset utility ────────────────────────────────────────────────
void resetMPUFIFO() {
    Wire.beginTransmission(0x68);
    Wire.write(0x6A);
    Wire.write(0x42);
    Wire.endTransmission();
}
