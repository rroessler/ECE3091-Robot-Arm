#include <common.h>

#include <SearchPath.h>

static int maxSearchStates = 1;

void update_search_angle_base(bool side) {
    if (jointAngles[0] == 0) {
        jointAngles[0] == 65;
        return;
    }

    if (jointAngles[0] == 180) {
        jointAngles[0] = 115;
        return;
    }

    jointAngles[0] = side ? 180 : 0;
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