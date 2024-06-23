
#include "Arduino.h"
#include <ICM20948_WE.h>
#include <Wire.h>
#include "KalmanFilter.h"
#include "Filter.h"

#define ICM20948_ADDR 0x68

KalmanFilter kalmanX;

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

bool dir = false;
double V = 0;
uint64_t timer = 0;
const FilterType type = EXPONENT_SMOOTH;

void imuSetup(ICM20948_WE& imu)
{
    if (! imu.init())
    {
        Serial.println("ICM20948 does not respond");
    }
    else
    {
        Serial.println("ICM20948 is connected");
    }
    Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
    imu.autoOffsets();
    Serial.println("Done!");

    imu.setAccRange(ICM20948_ACC_RANGE_2G);

    /*  Choose a level for the Digital Low Pass Filter or switch it off.
     *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7,
     * ICM20948_DLPF_OFF
     *
     *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle
     * mode!
     *
     *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
     *    0              246.0               1125/(1+ASRD) (default)
     *    1              246.0               1125/(1+ASRD)
     *    2              111.4               1125/(1+ASRD)
     *    3               50.4               1125/(1+ASRD)
     *    4               23.9               1125/(1+ASRD)
     *    5               11.5               1125/(1+ASRD)
     *    6                5.7               1125/(1+ASRD)
     *    7              473.0               1125/(1+ASRD)
     *    OFF           1209.0               4500
     *
     *    ASRD = Accelerometer Sample Rate Divider (0...4095)
     *    You achieve lowest noise using level 6
     */
    imu.setAccDLPF(ICM20948_DLPF_6);

    /*  Acceleration sample rate divider divides the output rate of the
     * accelerometer. Sample rate = Basic sample rate / (1 + divider) It can
     * only be applied if the corresponding DLPF is not off! Divider is a number
     * 0...4095 (different range compared to gyroscope) If sample rates are set
     * for the accelerometer and the gyroscope, the gyroscope sample rate has
     * priority.
     */
    imu.setAccSampleRateDivider(10);
}

void resetValue()
{
    // отсановка и сброс всех параметров перед возвращением в начальную позицию
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    dir = ! dir;
    kalmanX.reset();
    V = 0;
    delay(500);
    timer = micros();
}

void setup()
{
    pinMode(6, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(13, OUTPUT);

    digitalWrite(7, HIGH);
    digitalWrite(4, HIGH);

    Wire.begin();
    Serial.begin(115200);

    imuSetup(myIMU);
    timer = millis();
}

void loop()
{
    static auto timeForPrint = millis();
    static double filteredValue = 0;
    static double distance = 0;

    myIMU.readSensor();
    xyzFloat gVal = myIMU.getGValues();

    auto dt = micros() - timer;
    timer = micros();

    V += filteredValue * dt * 9.8 / 1000;
    distance += V * dt / 1000000;

    static int targetDist = 3000;

    // модель скорости в форме синуса, период которого равен полной длинне заданаго пути
    double distToRad = PI * abs(distance / targetDist);
    uint8_t speed = sin(distToRad) * 150 + 100;

    switch (type)
    {
        case NONE:
            filteredValue = gVal.y;
            break;

        case KALMAN:
            filteredValue = kalmanX.filter(gVal.y, speed - 100, dt);
            break;

        case AVRG:
            filteredValue = getAvrg(gVal.y, 10);
            break;

        case MEDIAN:
            filteredValue = getMedian(gVal.y);
            break;

        case EXPONENT_SMOOTH:
            filteredValue = getExpSmooth(gVal.y, 0.2);
            break;

        default:
            break;
    }

    if (! dir)
    {
        if (abs(distance) < targetDist * 0.9)
        {
            digitalWrite(7, LOW);
            digitalWrite(4, LOW);
            analogWrite(5, speed);
            analogWrite(6, speed);
        }
        else
            resetValue();
    }
    else
    {
        if (abs(distance) > targetDist * 0.01)
        {
            digitalWrite(7, HIGH);
            digitalWrite(4, HIGH);
            analogWrite(5, speed);
            analogWrite(6, speed);
        }
        else
            resetValue();
    }

    if (millis() - timeForPrint > 50)
    {
        Serial.print("accel - ");
        Serial.print(gVal.y);

        Serial.print("\tfilltered - ");
        Serial.print(filteredValue);

        Serial.print("\tV - ");
        Serial.print(V);

        Serial.print("\tDist - ");
        Serial.print(distance);

        timeForPrint = millis();
    }
}