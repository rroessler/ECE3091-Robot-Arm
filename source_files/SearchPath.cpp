#include <common.h>

#include <SearchPath.h>

static int maxSearchStates = 5;

void update_search_angle_base(bool side)
{
    if (jointAngles[0] == 0)
    {
        jointAngles[0] = 65;
        return;
    }

    if (jointAngles[0] == 180)
    {
        jointAngles[0] = 115;
        return;
    }

    jointAngles[0] = side ? 180 : 0;
}

int search_path_max_states()
{
    return maxSearchStates;
}

void update_search_angle_arm(int counter)
{
    switch (counter)
    {
    case 0:
        jointAngles[1] = 153;
        jointAngles[2] = -36;
        break;
    case 1:
        jointAngles[1] = 155;
        jointAngles[2] = -26;
        break;
    case 2:
        jointAngles[1] = 158;
        jointAngles[2] = -16;
        break;
    case 3:
        jointAngles[1] = 158;
        jointAngles[2] = -6;
        break;
    case 4:
        jointAngles[1] = 161;
        jointAngles[2] = 0;
        break;
    case 5:
        break;
    default:
        jointAngles[1] += 5;
        jointAngles[2] += 10;
        break;
    }
}