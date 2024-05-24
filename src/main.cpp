#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LedControl.h>
#include <Wire.h>
#include <EncButton.h>

#include "Timer.h"
#include "print3x5.hpp"

// Matrix
#define PIN_DATAIN 4
#define PIN_CLK 5 
#define PIN_CS 6 
#define MATRIX_A 0
#define MATRIX_B 1
#define PART_AMOUNT 55
// Define the number of devices (8x8 matrices) we have
#define NUM_DEVICES 2
// We have to invert the coordiantes because the matrices are rotated
#define BTN1_PIN 2
#define BTN2_PIN 3
#define invert(x) (7 - x)

typedef struct {
    int y;
    int x;
}Dir;

Button up(BTN1_PIN);
Button down(BTN2_PIN);
VirtButton dbl;

Adafruit_MPU6050 mpu;

// timer
Timer fall_tmr, disp_tmr;

// Create a new LedControl object
// Parameters: DIN pin, CLK pin, CS pin, number of devices
LedControl lc = LedControl(PIN_DATAIN, PIN_CLK, PIN_CS, NUM_DEVICES);

// LED matrixes
byte matrixA[8] = {0};
byte matrixB[8] = {0};

// How many seconds it takes for the sand to fall
int seconds = 60;

void setXY(byte matrix[], byte y, byte x, bool value);
byte getXY(byte matrix[], byte y, byte x);
void goDir(byte matrix[], byte y, byte x, Dir dir);
void getRightDirByAngle(Dir &dir, int angle);
void getLeftDirByAngle(Dir &dir, int angle);
void getDownDirByAngle(Dir &dir, int angle);
bool canGoDir(byte matrix[], byte y, byte x, Dir dir);
void updateMatrix(byte MatrixA, int angle);
void fall(int angle);
void fillMatrix(byte matrix[]);
void clearMatrix(byte matrix[]);
void changeTime(int amount);
void resetSand();

void fillMatrix(byte matrix[]) {
    for (int i = 1; i < 8; ++i) {
        matrix[i] = B11111111;
    }
}

void clearMatrix(byte matrix[]) {
    for (int i = 0; i < 8; ++i) {
        matrix[i] = 0;
    }
}

void resetSand() {
    clearMatrix(matrixA);
    clearMatrix(matrixB);
    fillMatrix(matrixA);
}

// Set a value on the matrix
void setXY(byte matrix[], byte y, byte x, bool value) {
    matrix[y] = value ? matrix[y] | (1 << x) : matrix[y] & ~(1 << x);
}

byte getXY(byte matrix[], byte y, byte x) {
    return (matrix[y] >> x) & 1;
}

void goDir(byte matrix[], byte y, byte x, Dir dir) {
    setXY(matrix, y, x, 0);
    setXY(matrix, y + dir.y, x + dir.x, 1);
}

void getDownDirByAngle(Dir &dir, int angle) {
    switch(angle) {
        case 0:
            dir = {1, 1};
            break;
        case 1:
            dir = {1, 0};
            break;
        case 2:
            dir = {1, 0};
            break;
        case 3:
            dir = {1, 0};
            break;
        case 4:
            dir = {1, -1};
            break;
        case 5:
            dir = {0, -1};
            break;
        case 6:
            dir = {0, -1};
            break;
        case 7:
            dir = {0, -1};
            break;
        case 8:
            dir = {-1, -1};
            break;
        case 9:
            dir = {-1, 0};
            break;
        case 10:
            dir = {-1, 0};
            break;
        case 11:
            dir = {-1, 0};
            break;
        case 12:
            dir = {-1, 1};
            break;
        case 13:
            dir = {0, 1};
            break;
        case 14:
            dir = {0, 1};
            break;
        case 15:
            dir = {0, 1};
            break;

    }
}

void getLeftDirByAngle(Dir &dir, int angle) {
    switch(angle) {
        case 0:
            dir = {1, 0};
            break;
        case 1:
            dir = {1, 1};
            break;
        case 2:
            dir = {100, 100};
            break;
        case 3:
            dir = {0, -1};
            break;
        case 4:
            dir = {1, 0};
            break;
        case 5:
            dir = {1, -1};
            break;
        case 6:
            dir = {100, 100};
            break;
        case 7:
            dir = {1, -1};
            break;
        case 8:
            dir = {0, -1};
            break;
        case 9:
            dir = {-1, -1};
            break;
        case 10:
            dir = {100, 100};
            break;
        case 11:
            dir = {-1, 1};
            break;
        case 12:
            dir = {-1, 0};
            break;
        case 13:
            dir = {-1, 1};
            break;
        case 14:
            dir = {100, 100};
            break;
        case 15:
            dir = {1, 1};
            break;
    }
}
void getRightDirByAngle(Dir &dir, int angle) {
    switch(angle) {
        case 0:
            dir = {0, 1};
            break;
        case 1:
            dir = {100, 100};
            break;
        case 2:
            dir = {100, 100};
            break;
        case 3:
            dir = {1, -1};
            break;
        case 4:
            dir = {0, 1};
            break;
        case 5:
            dir = {100, 100};
            break;
        case 6:
            dir = {100, 100};
            break;
        case 7:
            dir = {-1, 1};
            break;
        case 8:
            dir = {-1, 0};
            break;
        case 9:
            dir = {100, 100};
            break;
        case 10:
            dir = {100, 100};
            break;
        case 11:
            dir = {0, 1};
            break;
        case 12:
            dir = {0, 1};
            break;
        case 13:
            dir = {-1, 0};
            break;
        case 14:
            dir = {100, 100};
            break;
        case 15:
            dir = {1, 0};
            break;
    }
}

bool canGoDir(byte matrix[], byte y, byte x, Dir dir) {
    if (y + dir.y < 0 || y + dir.y > 7 || x + dir.x < 0 || x + dir.x > 7) {
        return false;
    }
    if (getXY(matrix, y + dir.y, x + dir.x)) {
        return false;
    }
    return true;
}

void fall(int angle) {
    if (fall_tmr) {
        if (angle >= 117 && angle <= 260) {
            // The sand is falling from matrixB
            Serial.println("falling from matrixB");
            if (getXY(matrixB, 0, 0)) {
                Serial.println("in if statement");
                setXY(matrixB, 0, 0, 0);
                setXY(matrixA, 7, 7, 1);
            }
        } else {
            // The sand is falling from matrixA
            if (getXY(matrixA, 7, 7)) {
                setXY(matrixA, 7, 7, 0);
                setXY(matrixB, 0, 0, 1);
            }
        }
    }
}

void updateMatrix(byte matrix[], int angle) {
    Dir downDir, leftDir, rightDir;
    getDownDirByAngle(downDir, angle);
    getLeftDirByAngle(leftDir, angle);
    getRightDirByAngle(rightDir, angle);
    randomSeed(analogRead(0)); 

    uint8_t r = random(3, 20);
    for (uint8_t k = 0; k < r; k++) {
        for (uint16_t i = k; i < 64; i += r) {
            uint8_t y = i / 8;
            uint8_t x = i - y * 8;  // i % w
            if (getXY(matrix, y, x) == 0) {
                continue;
            }
            bool canGoDown = canGoDir(matrix, y, x, downDir);
            bool canGoLeft = canGoDir(matrix, y, x, leftDir);
            bool canGoRight = canGoDir(matrix, y, x, rightDir);

            // If we can go down, then we go down
            if (canGoDown) {
                goDir(matrix, y, x, downDir);
            } else {
                // Else we want to go either left or right with 50% chance
                if (canGoLeft && canGoRight) {
                    if (random(0, 2) == 0) {
                        goDir(matrix, y, x, leftDir);
                    } else {
                        goDir(matrix, y, x, rightDir);
                    }
                } else if (canGoLeft) {
                    goDir(matrix, y, x, leftDir);
                } else if (canGoRight) {
                    goDir(matrix, y, x, rightDir);
                }
            }
        }
    }
}

void printDig(byte matrix[], int Y, int X, uint8_t dig) {
    if (dig > 9) return;
    for (uint8_t y = 0; y < 5; y++) {
        Serial.println("line: " + y);
        for (uint8_t x = 0; x < 3; x++) {
            if ((font3x5[dig][y] & (1 << (2 - x))) != 0) {
                setXY(matrix,Y + y,X + x, 1);
                Serial.print("1 ");
            } else {

                Serial.print("0 ");
            }
        }
        Serial.println();
    }
}

void changeTime(int amount) {
    disp_tmr.setTimeout(3000);
    //clear the matrixes
    // mtrx.clear();
    clearMatrix(matrixA);
    clearMatrix(matrixB);

    seconds += amount;
    if (seconds < 0) seconds = 0;
    uint8_t min = seconds / 60;
    uint8_t sec = seconds % 60;

    printDig(matrixA, 1, 0, min / 10);
    printDig(matrixA, 1, 4, min % 10);
    printDig(matrixB, 1, 0, sec / 10);
    printDig(matrixB, 1, 4, sec % 10);

    fall_tmr.setInterval(seconds * 1000ul / PART_AMOUNT);
    // resetSand();
    // here is just a print function
    // mtrx.update();

}

void checkButtons() {
    up.tick();
    down.tick();
    dbl.tick(up, down);

    if (dbl.click()) resetSand();

    if (up.click()) changeTime(1);
    if (up.step(0)) changeTime(10);

    if (down.click()) changeTime(-1);
    if (down.step(0)) changeTime(-10);

}

void setup() {
    Serial.begin(115200);
    // wait for serial port to open, needed for native USB
    while (!Serial) {
        delay(10); 
    }
    Serial.print("Init");

    // Try to initialize the MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Initialize each MAX7219 module
    for (int i = 0; i < NUM_DEVICES; i++) {
        lc.shutdown(i, false);    // Wake up displays
        lc.setIntensity(i, 1);    // Set brightness level (0 is min, 15 is max)
        lc.clearDisplay(i);         // Clear display register
    }
    Serial.println("Device count: " + String(lc.getDeviceCount()));

    randomSeed(analogRead(0));  // Pin A0 is typically used, but it can be any analog pin

    fillMatrix(matrixA);
    // printDig(matrixA, 1, 2, 7);
    fall_tmr.setInterval(seconds * 1000ul / PART_AMOUNT);
    disp_tmr.attach(resetSand);
}


void loop() {
 // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the angle on the XZ plane
    float accel_x = a.acceleration.x;
    float accel_z = a.acceleration.y;

    float angle_radians = atan2(accel_x, accel_z);
    float angle_degrees = angle_radians * 180 / PI;
    angle_degrees += 40;

    // Ensure the angle is between 0 and 360 degrees
    if (angle_degrees < 0) {
        angle_degrees += 360;
    }

    // Print the angle
    Serial.println(angle_degrees);

    // // Display pattern1 on the first matrix (index 0)
    int angle = (int)(angle_degrees / 22.5f);

    Serial.println("angle: ");
    Serial.println(angle_degrees);

    disp_tmr.tick();
    checkButtons();

    if (!disp_tmr.state()) {
        updateMatrix(matrixA, angle);
        updateMatrix(matrixB, angle);

        fall(angle_degrees);
    }

    //Display the matrixes
    for (byte row = 0; row < 8; row++) {
        lc.setRow(MATRIX_A, invert(row), matrixA[row]);
    }
    // lc.setRow(MATRIX_B, invert(0), matrixA[0]);
    for (byte row = 0; row < 8; row++) {
        lc.setRow(MATRIX_B, invert(row), matrixB[row]);
    }
    delay(90);
}