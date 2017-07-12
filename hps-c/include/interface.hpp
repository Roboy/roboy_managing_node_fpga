#pragma once

#include <ncurses.h>
#include <vector>
#include <cstring>
#include <unistd.h>
#include "myoControl.hpp"

using namespace std;

class Interface {
public:
    Interface(vector<int32_t*> &myo_base);
    Interface(vector<int32_t*> &myo_base, vector<int32_t*> &i2c_base);

    ~Interface();

    void printMessage(uint row, uint col, char *msg);

    void printMessage(uint row, uint col, char *msg, uint color);

    void print(uint row, uint startcol, uint length, const char *s);

    void clearAll(uint row);

    void querySensoryData();

    void processing(char *msg1, char *what, char *msg2);

    void processing(char *msg1, char *msg2);

    void toggleSPI();

    void reset();

    void setGains();

    void positionControl();

    void velocityControl();

    void displacementControl();

    void switchMotor();

    void zeroWeight();

    void setAllTo();

    void estimateSpringParameters();

    void recordTrajectories();

    void playTrajectories();

    MyoControl *myoControl;
    uint timeout_ms = 10;
private:
    uint rows, cols;
    int32_t pos;
    uint ganglion_id = 0;
    uint motor_id = 0;
    char inputstring[30];
};
