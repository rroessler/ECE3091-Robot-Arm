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


/*
*   FUNCTION:
*       Initialises the required colour pins as per using Arduino's
*       pinMode function.
*   
*   INPUT:
*       nil. Although uses the pins defined within this file above.
*
*   OUPUT:
*       nil.
*/
void init_colour_sensor_pins()
{
    for (int i = 0; i <= 2; i++)
    {
        pinMode(ledArray[i], OUTPUT);
    }
}


/*
*   FUNCTION:
*       Reads the colour sensor (LDR) values (RED, GREEN and BLUE) a given 
*       number of times.
*   
*   INPUT:
*       int times -> Number of times to read the LDR colour sensor.
*
*   OUPUT:
*       nil. However changes the avgRead placeholder integer.
*/
void read_colour_sensor(int times)
{
    int reading = 0;
    int sum = 0;

    for (int i = 0; i < times; i++)
    {
        // take analog read of the sensor pin and add to sum
        reading = analogRead(sensorPIN);
        sum += reading;
        delay(10);
    }

    // update the average read placeholder
    avgRead = sum / times;
}


/*
*   FUNCTION:
*       Helper function to intiliase the white and black sample readings.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Sets the whiteArray[] and blackArray[] values as per the readings
*       determined.
*/
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


/*
*   FUNCTION:
*       Checks a colour reading from the colour sensor, as per required for the 
*       type of checking.
*           0 - Black Sampling
*           1 - White Sampling
*           Default - General Colour Sensing
*   
*   INPUT:
*       int type -> The type of colour sensing to conduct.
*
*   OUPUT:
*       nil. Outputs to the placeholder colourArray[] float values.
*/
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


/*
*   FUNCTION:
*       Conducts a block colour check through running the the determine_block_colour()
*       multiple times and finding the max index received that correlates to the determined
*       colour.
*   
*   INPUT:
*       int times -> The number of times to run the colour checking
*
*   OUPUT:
*       int maxIndex -> The index corresponding the the colour found most:
*                           0 - Red, 1 - Green, 2 - Blue
*/
int run_block_colour_multi_check(int times)
{
    // placeholder for counting the number of times each colour is determined
    int counters[] = {0, 0, 0};

    // iterating over the number of times
    for (int i = 0; i < times; i++)
    {   
        // determining the block colour and incrementing the corresponding counter
        int currentVal = determine_block_colour();
        if (currentVal != -1)
        {
            counters[currentVal] += 1;
        }
    }

    // now we find the max index of counters[] and return
    int maxIndex = 0;
    for (int i = 0; i <= 2; i++) {
        if (counters[maxIndex] < counters[i]) {
            maxIndex = i;
        }
    }

    // and return
    return maxIndex;
}


/*
*   FUNCTION:
*       Determines the current block colour being read.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       int -> Value corresponding to the colour that was read.
*/
int determine_block_colour()
{
    // update the colour sensor with new read colour data
    check_colour();

    // debug
    Serial.println("RED: " + String(colourArray[0]));
    Serial.println("GREEN: " + String(colourArray[1]));
    Serial.println("BLUE: " + String(colourArray[1]));
    Serial.println();

    // if dominant red then must be red
    if (colourArray[0] > 100 && colourArray[1] < 80 && colourArray[2] < 80)
    {
        // red
        return 0;
    }

    // if all values are high then must be green
    if (colourArray[1] > 125)
    {
        // grean
        return 1;
    }

    // if dominant blue then must be blue
    if (colourArray[0] < 80 && colourArray[1] < 80 && colourArray[2] > 80)
    {
        // blue
        return 2;
    }

    return -1; // return unknown as -1
}