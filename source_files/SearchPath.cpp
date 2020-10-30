#include <common.h>

#include <SearchPath.h>

static int maxSearchStates = 4;

void update_search_angle_base(bool side) {
    if (jointAngles[0] == 180 - side ? 0 : 180) {
        // based on side the angle was set at 0 or 180, thus change to 90
        jointAngles[0] = 90;
    } else {
        // otherwise set to chosen side max
        jointAngles[0] = side ? 180 : 0;
    }
}

int search_path_max_states() {
    return maxSearchStates;
}

void update_search_angle_arm(int counter) {
    switch (counter) {
        default:
            jointAngles[1] -= 13;
            jointAngles[2] -= 13;
            break;
    }
}