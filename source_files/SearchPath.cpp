#include <common.h>

#include <SearchPath.h>

// define the max number of search states for the searching determination
static int maxSearchStates = 5;


/*
*   FUNCTION:
*       Updates the base servo angle to move to. Relies on the current side
*       in which the storage box is on and changes to base angle value to 
*       its opposite side. This results in a "sweeping" like motion, as opposed
*       to moving to a position.
*   
*   INPUT:
*       bool side -> Boolean to specify the current side of storage box
*
*   OUPUT:
*       nil.
*/
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


/*
*   FUNCTION:
*       Returns the max search states value found in this file.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       int maxSearchStates -> Maximum Number of Search States
*/
int search_path_max_states()
{
    return maxSearchStates;
}


/*
*   FUNCTION:
*       Based on the current search paths counter, sets the new arm and wrist
*       angles for the robot to move to. This moves the arm outwards on each
*       count increment.
*
*       Currently does so as a hardcoded path that was found to be best, however 
*       another impementation (see: default case), shows how this could have been
*       done programatically, if the arm angles were more accurate.
*   
*   INPUT:
*       int counter -> Counter value to align with point in which the robot
*       arm is at in its search path.
*
*   OUPUT:
*       nil.
*/
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
        jointAngles[1] = 160;
        jointAngles[2] = -20;
        break;
    case 3:
        jointAngles[1] = 168;
        jointAngles[2] = -17;
        break;
    case 4:
        jointAngles[1] = 174;
        jointAngles[2] = -8;
        break;
    default:
        jointAngles[1] += 5;
        jointAngles[2] += 5;
        break;
    }
}