#include <Arduino.h>

#include <ProximitySensor.h>

/*
*   FUNCTION:
*       Reads the proximity sensor analog value from the proximity
*       Arduino PIN given.
*   
*   INPUT:
*       int proximityPIN -> PIN to read analogue value from
*
*   OUPUT:
*       int val -> Analog value that was read from the given pin
*/
int read_proximity_sensor(int proximityPIN)
{
    return analogRead(proximityPIN);
}


/*
*   FUNCTION:
*       Takes a proximity average from a read count value given.
*       Iterates over the readCount and takes the average of
*       each analog read of the proximity pin.
*   
*   INPUT:
*       int proximityPIN -> PIN to read analogue value from
*       int readCount -> Number of times to read the proximity PIN
*
*   OUPUT:
*       int val -> the average of reading values
*/
int read_proximity_average(int proximityPIN, int readCount)
{
    int sum = 0;
    for (int i = 0; i < readCount; i++)
    {
        sum += read_proximity_sensor(proximityPIN);
        delay(10);
    }

    return sum / readCount;
}


/*
*   FUNCTION:
*       Uses the values from the read_proximity_average() and a given currentAverage and threshold
*       value to determine if a block has been detected.
*   
*   INPUT:
*       int proximityPIN -> PIN to read analogue value from
*       int currentCounter -> External counter variable that increases on reading a block
*       int currentAverage -> Given average that was presumable found before detecting
*       int threshold -> Threshold value for finding blocks
*
*   OUPUT:
*       int currentCounter -> The counter value given as an input incremented if a block is detected
*/
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