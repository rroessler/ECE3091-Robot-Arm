#include <Arduino.h>

#include <ProximitySensor.h>

int read_proximity_sensor(int proximityPIN)
{
    return analogRead(proximityPIN);
}

int read_proximity_average(int proximityPIN, int readCount)
{
    int sum = 0;
    for (int i = 0; i < readCount; i++)
    {
        sum += analogRead(proximityPIN);
        delay(10);
    }

    return sum / readCount;
}

int detect_blocks(int proximityPIN, int currentCounter, int currentAverage, int threshold)
{
    int val = read_proximity_sensor(proximityPIN);

    if (currentAverage - val >= threshold)
    {
        // Serial.println("BLOCK DETECTED! - " + String(val));
        return currentCounter + 1;
    }

    // Serial.println("Nah - " + String(val));
    return currentCounter;
}