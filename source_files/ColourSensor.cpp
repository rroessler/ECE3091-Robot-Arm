#include <Arduino.h>

#include <ColourSensor.h>

// define colour sensor pins and analogue input
const int ledArray[] = {2, 3, 4};
// { red, green, blue }
const int sensorPIN = 1;

// placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;

// floats to hold for colour arrays
float colourArray[] = {0, 0, 0};
float whiteArray[] = {0, 0, 0};
float blackArray[] = {0, 0, 0};

// placeholder for the average read
int avgRead;

void init_colour_sensor_pins()
{
    for (int i = 0; i <= 2; i++)
    {
        pinMode(ledArray[i], OUTPUT);
    }
}

void read_colour_sensor(int times)
{
    int reading = 0;
    int sum = 0;

    for (int i = 0; i < times; i++)
    {
        reading = analogRead(sensorPIN);
        sum += reading;
        delay(10);
    }

    // update the average read placeholder
    avgRead = sum / times;
}

void set_start_balance()
{
    delay(5000); // delay for 5 seconds to give time to give white sample

    // *** scanning white sample
    // run through each light and get a standard white reading
    check_colour(1);

    delay(5000); // delay for 5 seconds to give time to get black sample ready

    // *** scanning black sample
    // again run through each light and get a standard black reading
    check_colour(0);
}

void check_colour(int type)
{
    for (int i = 0; i <= 2; i++)
    {
        digitalWrite(ledArray[i], HIGH);
        delay(100);
        read_colour_sensor(5);

        switch (type)
        {
        case 0:
            // black reading
            blackArray[i] = avgRead;
            break;
        case 1:
            // white reading
            whiteArray[i] = avgRead;
            break;
        default:
            colourArray[i] = avgRead;
            float greyDiff = whiteArray[i] - blackArray[i];
            colourArray[i] = (colourArray[i] - blackArray[i]) / (greyDiff)*255;
            break;
        }

        digitalWrite(ledArray[i], LOW);
        delay(100);
    }
}

int run_block_colour_multi_check(int times)
{
    int counters[] = {0, 0, 0};

    for (int i = 0; i < times; i++)
    {
        int currentVal = determine_block_colour();
        if (currentVal != -1)
        {
            counters[determine_block_colour()] += 1;
        }

        // delay(100);
    }

    // now we find the max index of counters[] and return
    int maxIndex = 0;
    for (int i = 0; i <= 2; i++) {
        if (counters[maxIndex] < counters[i]) {
            maxIndex = i;
        }
    }

    return maxIndex;
}

int determine_block_colour()
{
    check_colour();

    // if dominant red then must be red
    if (colourArray[0] > 100 && colourArray[1] < 80 && colourArray[2] < 80)
    {
        return 0;
    }

    // if all values are high then must be green
    if (colourArray[1] > 125)
    {
        return 1;
    }

    // if dominant blue then must be blue
    if (colourArray[0] < 80 && colourArray[1] < 80 && colourArray[2] > 80)
    {
        return 2;
    }

    return -1; // return unknown through -1
}