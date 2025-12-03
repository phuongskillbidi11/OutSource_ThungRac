#include "MotorController.h"

// Define global variables
Motor motors[10];
int motorCount = 0;
bool motorsReady = false;
int speedPWM[5] = {0, 64, 128, 192, 255};