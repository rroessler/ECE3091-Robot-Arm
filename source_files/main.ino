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
    Serial.println();
    Serial.println("State: " + String(currentState));
    Serial.println();
    // additional debug states to handle mixed states whilst testing things
    // don't forget to delete/change state changes from these on final version
    switch (currentState)
    {
    case 0:
        // Initialisation
        initialise_robot();
        break;
    case 1:
        // Search for Blocks on Path
        run_search_path();
        break;
    case 2:
        // Found Block so Determine Size
        break;
    case 3:
        // Pickup Block
        start_pickup_routine();
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
        Serial.println("Yay!");
        break;
    case 10:
        // Debug 1
        handle_debug_input();
        break;
    case 11:
        // Debug 2
        break;
    }

    update_state();

    // slight delay whilst debugging
    delay(1000);
}
