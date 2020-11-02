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
    // additional debug states to handle mixed states whilst testing things
    // don't forget to delete/change state changes from these on final version
    switch (currentState)
    {
    case 0:
        // Initialisation
        initialise_robot();
        initialise_colour_sensor();
        break;
    case 1:
        // determine storage box location
        storage_search_path();
        break;
    case 2:
        // Search for Blocks on Path
        blocks_search_path();
        break;
    case 3:
        // Pickup Block
        start_pickup_routine();
        break;
    case 4:
        // Move to Colour Sensor and Determine Colour
        move_to_colour_sensor();
        find_block_colour();
        break;
    case 5:
        // Place in Storage
        place_block_in_storage();
        break;
    case 6:
        // Return to Search
        return_to_search();
        break;
    case 7:
        // Dropped Block (accident)
        break;
    case 8:
        // Reset
        break;
    case 9:
        // Finish
        // Serial.println("Yay!");
        break;
    case 10:
        // Lost
        Serial.println("LOST!");
        break;
    case 11:
        // Debug 1
        handle_debug_input();
        break;
    case 12:
        // Debug 2
        manual_change_state();
        break;
    case 13:
        // Debug 3
        storage_placement_test();
        break;
    }

    update_state();

    // slight delay whilst debugging
    delay(1000);
}
