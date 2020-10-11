#include <StateRoutines.h>
#include <common.h>

void setup()
{
    // open serial port for debugging
    Serial.begin(9600);

    // set the current state to initialise
    currentState = 0;
}

void loop()
{
    switch (currentState)
    {
    case 0:
        // Initialisation
        initialise_robot();
        break;
    case 1:
        // Search for Blocks on Path
        Serial.println("Yay!");
        delay(1000);
        break;
    case 2:
        // Found Block so Determine Size
        break;
    case 3:
        // Pickup Block
        break;
    case 4:
        // Move to Colour Sensor and Determine Colour
        break;
    case 5:
        // Place in Storage
        break;
    case 6:
        // Return to Search
        break;
    case 7:
        // Dropped Block (accident)
        break;
    case 8:
        // Reset
        break;
    case 9:
        // Finish
        break;
    }

    update_state();
}