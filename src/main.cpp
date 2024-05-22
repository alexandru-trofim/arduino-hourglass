#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LedControl.h>
#include <Wire.h>

// Matrix
#define PIN_DATAIN 5
#define PIN_CLK 6
#define PIN_CS 4
#define MATRIX_A 0
#define MATRIX_B 1
// Define the number of devices (8x8 matrices) we have
#define NUM_DEVICES 2
// We have to invert the coordiantes because the matrices are rotated
#define invert(x) (7 - x)

typedef struct {
    int y;
    int x;
}Dir;

Adafruit_MPU6050 mpu;
int ok = 0;

// Create a new LedControl object
// Parameters: DIN pin, CLK pin, CS pin, number of devices
LedControl lc = LedControl(PIN_DATAIN, PIN_CLK, PIN_CS, NUM_DEVICES);
byte matrixA[8] = {0};
byte matrixB[8] = {0};

void setXY(byte matrix[], byte y, byte x, bool value);
byte getXY(byte matrix[], byte y, byte x);
void goDir(byte matrix[], byte y, byte x, Dir dir);
void getRightDirByAngle(Dir &dir, int angle);
void getLeftDirByAngle(Dir &dir, int angle);
void getDownDirByAngle(Dir &dir, int angle);
bool canGoDir(byte matrix[], byte y, byte x, Dir dir);
void updateMatrix(byte MatrixA, int angle);

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
        // case 0:
        //     // .   {y, x}
        //     dir = {1, 1};
        //     break;
        // case 5:
        //     dir = {0, 1};
        //     break;
        // case 6:
        //     dir = {-1, 1};
        //     break;
        // case 7:
        //     dir = {-1, 0};
        //     break;
        // case 4:
        //     dir = {-1, -1};
        //     break;
        // case 1:
        //     dir = {0, -1};
        //     break; 
        // case 2:
        //     dir = {1, -1};
        //     break;
        // case 3:
        //     dir = {1, 0};
        //     break;
        case 0:
            // .   {y, x}
            dir = {1, 1};
            break;
        case 1:
            // .   {y, x}
            dir = {1, 0};
            break;
        case 2:
            // .   {y, x}
            dir = {1, 0};
            break;
        case 3:
            // .   {y, x}
            dir = {1, 0};
            break;
        case 4:
            // .   {y, x}
            dir = {1, -1};
            break;
        case 5:
            // .   {y, x}
            dir = {0, -1};
            break;
        case 6:
            // .   {y, x}
            dir = {0, -1};
            break;
        case 7:
            // .   {y, x}
            dir = {0, -1};
            break;
        case 8:
            // .   {y, x}
            dir = {-1, -1};
            break;
        case 9:
            // .   {y, x}
            dir = {-1, 0};
            break;
        case 10:
            // .   {y, x}
            dir = {-1, 0};
            break;
        case 11:
            // .   {y, x}
            dir = {-1, 0};
            break;
        case 12:
            // .   {y, x}
            dir = {-1, 1};
            break;
        case 13:
            // .   {y, x}
            dir = {0, 1};
            break;
        case 14:
            // .   {y, x}
            dir = {0, 1};
            break;
        case 15:
            // .   {y, x}
            dir = {0, 1};
            break;

    }
}

void getLeftDirByAngle(Dir &dir, int angle) {
    switch(angle) {
        // case 0:
        //     // .   {y, x}
        //     dir = {1, 0};
        //     break;
        // case 5:
        //     dir = {1, 1};
        //     break;
        // case 6:
        //     dir = {0, 1};
        //     break;
        // case 7:
        //     dir = {-1, 1};
        //     break;
        // case 4:
        //     dir = {-1, 0};
        //     break;
        // case 1:
        //     dir = {-1, -1};
        //     break; 
        // case 2:
        //     dir = {0, -1};
        //     break;
        // case 3:
        //     dir = {1, -1};
        //     break;
        case 0:
            // .   {y, x}
            dir = {1, 0};
            break;
        case 1:
            // .   {y, x}
            dir = {1, 1};
            break;
        case 2:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 3:
            // .   {y, x}
            dir = {0, -1};
            break;
        case 4:
            // .   {y, x}
            dir = {1, 0};
            break;
        case 5:
            // .   {y, x}
            dir = {1, -1};
            break;
        case 6:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 7:
            // .   {y, x}
            dir = {1, -1};
            break;
        case 8:
            // .   {y, x}
            dir = {0, -1};
            break;
        case 9:
            // .   {y, x}
            dir = {-1, -1};
            break;
        case 10:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 11:
            // .   {y, x}
            dir = {-1, 1};
            break;
        case 12:
            // .   {y, x}
            dir = {-1, 0};
            break;
        case 13:
            // .   {y, x}
            dir = {-1, 1};
            break;
        case 14:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 15:
            // .   {y, x}
            dir = {1, 1};
            break;
    }
}
void getRightDirByAngle(Dir &dir, int angle) {
    switch(angle) {
        // case 0:
        //     // .   {y, x}
        //     dir = {0, 1};
        //     break;
        // case 5:
        //     dir = {-1, 1};
        //     break;
        // case 6:
        //     dir = {-1, 0};
        //     break;
        // case 7:
        //     dir = {-1, -1};
        //     break;
        // case 4:
        //     dir = {0, -1};
        //     break;
        // case 1:
        //     dir = {1, -1};
        //     break; 
        // case 2:
        //     dir = {1, 0};
        //     break;
        // case 3:
        //     dir = {1, 1};
        //     break;
        case 0:
            // .   {y, x}
            dir = {0, 1};
            break;
        case 1:
            dir = {100, 100};
            break;
        case 2:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 3:
            // .   {y, x}
            dir = {1, -1};
            break;
        case 4:
            // .   {y, x}
            dir = {0, 1};
            break;
        case 5:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 6:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 7:
            // .   {y, x}
            dir = {-1, 1};
            break;
        case 8:
            // .   {y, x}
            dir = {-1, 0};
            break;
        case 9:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 10:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 11:
            // .   {y, x}
            dir = {0, 1};
            break;
        case 12:
            // .   {y, x}
            dir = {0, 1};
            break;
        case 13:
            // .   {y, x}
            dir = {-1, 0};
            break;
        case 14:
            // .   {y, x}
            dir = {100, 100};
            break;
        case 15:
            // .   {y, x}
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

void updateMatrix(byte MatrixA[], int angle) {
    Dir downDir, leftDir, rightDir;
    getDownDirByAngle(downDir, angle);
    getLeftDirByAngle(leftDir, angle);
    getRightDirByAngle(rightDir, angle);

        uint8_t r = random(3, 20);
        for (uint8_t k = 0; k < r; k++) {
            for (uint16_t i = k; i < 64; i += r) {
            uint8_t y = i / 8;
            uint8_t x = i - y * 8;  // i % w
        // for (int y = 7; y >= 0; y--) {
        //     for (int x = 7; x >= 0; x--) {
            // Ok so we have the x and y coordinates
            // Now we need to check if we can move down, left and right 
            if (getXY(matrixA, y, x) == 0) {
                continue;
            }
            bool canGoDown = canGoDir(matrixA, y, x, downDir);
            bool canGoLeft = canGoDir(matrixA, y, x, leftDir);
            bool canGoRight = canGoDir(matrixA, y, x, rightDir);

            // If we can go down, then we go down
            if (canGoDown) {
                goDir(matrixA, y, x, downDir);
                // Serial.println("down dir: "); 
                // Serial.println(downDir.y);
                // Serial.println(downDir.x);
            } else {
                // Else we want to go either left or right with 50% chance
                if (canGoLeft && canGoRight) {
                    if (random(0, 2) == 0) {
                        goDir(matrixA, y, x, leftDir);
                        // Serial.println("going left");
                    } else {
                        goDir(matrixA, y, x, rightDir);
                        // Serial.println("going right");
                    }
                } else if (canGoLeft) {
                    // Serial.println("going left");
                    goDir(matrixA, y, x, leftDir);
                } else if (canGoRight) {
                    // Serial.println("going right");
                    goDir(matrixA, y, x, rightDir);
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    // wait for serial port to open, needed for native USB
    while (!Serial) {
        delay(10); 
    }

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

    matrixA[0] = 0b11111111;
    matrixA[1] = 0b00001111;
    matrixA[2] = 0b11111100;
    matrixA[3] = 0b11111110;
    delay(100);
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

    // Display pattern1 on the first matrix (index 0)
    int angle = (int)(angle_degrees / 22.5f);
    Serial.println("angle: ");
    Serial.println(angle);
    updateMatrix(matrixA, angle);
    updateMatrix(matrixA, angle);
    // Dir downDir, leftDir, rightDir;
    // getDownDirByAngle(downDir, angle);
    // getLeftDirByAngle(leftDir, angle);
    // getRightDirByAngle(rightDir, angle);

    //     uint8_t r = random(3, 20);
    //     for (uint8_t k = 0; k < r; k++) {
    //         for (uint16_t i = k; i < 64; i += r) {
    //         uint8_t y = i / 8;
    //         uint8_t x = i - y * 8;  // i % w
    //     // for (int y = 7; y >= 0; y--) {
    //     //     for (int x = 7; x >= 0; x--) {
    //         // Ok so we have the x and y coordinates
    //         // Now we need to check if we can move down, left and right 
    //         if (getXY(matrixA, y, x) == 0) {
    //             continue;
    //         }
    //         bool canGoDown = canGoDir(matrixA, y, x, downDir);
    //         bool canGoLeft = canGoDir(matrixA, y, x, leftDir);
    //         bool canGoRight = canGoDir(matrixA, y, x, rightDir);

    //         // If we can go down, then we go down
    //         if (canGoDown) {
    //             goDir(matrixA, y, x, downDir);
    //             // Serial.println("down dir: "); 
    //             // Serial.println(downDir.y);
    //             // Serial.println(downDir.x);
    //         } else {
    //             // Else we want to go either left or right with 50% chance
    //             if (canGoLeft && canGoRight) {
    //                 if (random(0, 2) == 0) {
    //                     goDir(matrixA, y, x, leftDir);
    //                     // Serial.println("going left");
    //                 } else {
    //                     goDir(matrixA, y, x, rightDir);
    //                     // Serial.println("going right");
    //                 }
    //             } else if (canGoLeft) {
    //                 // Serial.println("going left");
    //                 goDir(matrixA, y, x, leftDir);
    //             } else if (canGoRight) {
    //                 // Serial.println("going right");
    //                 goDir(matrixA, y, x, rightDir);
    //             }
    //         }
    //     }
    // }
    // for (int i = 0 ; i < 8; i++) {
        // Serial.println(matrixA[i], BIN);
    // }

    // lc.setLed(1, invert(2), invert(0), 1);
    // Display pattern2 on the second matrix (index 1)

    for (byte row = 0; row < 8; row++) {
        lc.setRow(MATRIX_A, invert(row), matrixA[row]);
    }
    // lc.setLed(MATRIX_B, 0, 0, 1);

    delay(100);


}