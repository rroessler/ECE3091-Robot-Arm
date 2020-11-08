#ifndef _PROXIMITYSENSOR_H_
#define _PROXIMITYSENSOR_H_

// forward declaration to use functions where required
int read_proximity_sensor(int proximityPIN);
int read_proximity_average(int proximityPIN, int readCount = 15);

int detect_blocks(int proximityPIN, int currentCounter, int currentAverage, int threshold);

#endif