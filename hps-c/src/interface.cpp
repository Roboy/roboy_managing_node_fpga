#include "interface.hpp"

//! standard query messages
char welcomestring[] = "commandline tool for controlling myode muscle via de0-nano setup";
char commandstring[] = "[0]position, [1]velocity, [2]displacement, [3]switch motor, [4]zero weight, [5]allTo, [6]estimateSpringParams, [7]toggleSPI, [8]reset, [space]record, [enter]play, [/]pidParams, [9]exit";
char choosecontrolstring[] = "choose control mode (0: Position, 1:Velocity, 2: Force)?";
char setpointstring[] = "set point?";
char setpositionstring[] = "set position (ticks)?";
char setvelstring[] = "set velocity (ticks/s) ?";
char setdisplacementstring[] = "set displacement (ticks)?";
char kpstring[] = "Kp?";
char kdstring[] = "Kd?";
char kistring[] = "Ki?";
char motorstring[] = "which motor?";
char motorinfo[30];
char runningstring[] = "running ";
char recordingstring[] = "recording ";
char donestring[] = "done ";
char samplingtimestring[] = "samplingTime [milliseconds]: ";
char recordtimestring[] = "recordTime [seconds]: ";
char invalidstring[] = "invalid!";
char quitstring[] = " [hit q to quit]";
char averageconnectionspeedstring[] = "average connection speed: ";
char logfilestring[] = "see logfile measureConnectionTime.log for details";
char filenamestring[] = "enter filename to save recorded trajectories: ";
char remotecontrolactivestring[] = "remote control active [hit q to quit]";
char publishingmotorstring[] = "publishing motor status[hit q to quit]";
char receivedupdatestring[] = "received update";
char errormessage[] = "Error: received update for motor that is not connected";
char byebyestring[] = "BYE BYE!";

enum COLORS {
    CYAN = 1,
    RED,
    GREEN,
};

Interface::Interface(vector<int32_t*> &myo_base) {
	myoControl = new MyoControl(myo_base);
	//! start ncurses mode
	initscr();
	//! Start color functionality
    start_color();
	init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
	init_pair(RED, COLOR_RED, COLOR_BLACK);
	init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
	//! get the size of the terminal window
	getmaxyx(stdscr, rows, cols);

	print(0, 0, cols, "-");
	printMessage(1, 0, welcomestring);
	print(2, 0, cols, "-");
	print(7, 0, cols, "-");
	querySensoryData();
	printMessage(3, 0, commandstring);
}

Interface::Interface(vector<int32_t*> &myo_base, vector<int32_t*> &i2c_base){
	
}

Interface::~Interface() {
	delete myoControl;
	clearAll(0);
	printMessage(rows / 2, cols / 2 - strlen(byebyestring) / 2, byebyestring);
	refresh();
	usleep(1000000);
	endwin();
}

void Interface::printMessage(uint row, uint col, char *msg) {
	mvprintw(row, col, "%s", msg);
	refresh();
}

void Interface::printMessage(uint row, uint col, char *msg, uint color) {
	mvprintw(row, col, "%s", msg);
	mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
	refresh();
}

void Interface::print(uint row, uint startcol, uint length, const char *s) {
	for (uint i = startcol; i < startcol + length; i++) {
		mvprintw(row, i, "%s", s);
	}
	refresh();
}

void Interface::clearAll(uint row) {
	for (uint i = row; i < rows; i++) {
		print(i, 0, cols, " ");
	}
	refresh();
}

void Interface::querySensoryData() {
	int32_t pos = myoControl->getPosition(motor_id);
	int16_t vel = myoControl->getVelocity(motor_id);
	int16_t current = myoControl->getCurrent(motor_id);
	int16_t displacement = myoControl->getDisplacement(motor_id);
	int16_t pwm = myoControl->getPWM(motor_id);

	int row = 8;
	sprintf(motorinfo, "motor %d   ", motor_id);
	printMessage(row++, 0, motorinfo, CYAN);
	mvprintw(row++, 0, "pwm:                    %d\t\t 0x%032x        ", pwm, pwm);
	mvprintw(row++, 0, "actuatorPos (ticks):    %d\t\t 0x%032x        ", pos, pos);
	mvprintw(row++, 0, "actuatorPos (degree):   %f\t\t                ", pos*(2* 3.14159265359f / (2000.0f * 53.0f))*180/3.14159265359f);
	mvprintw(row++, 0, "actuatorVel:            %d\t\t 0x%032x        ", vel, vel);
	mvprintw(row++, 0, "actuatorCurrent:        %d\t\t 0x%032x        ", current, current);
	mvprintw(row++, 0, "tendonDisplacement:     %d\t\t 0x%032x        ", displacement, displacement);

	print(row++, 0, cols, "-");
	int Pgain, Igain, Dgain, forwardGain, deadband, setPoint, setPointMin, setPointMax;
	myoControl->getPIDcontrollerParams(Pgain, Igain, Dgain, forwardGain, deadband, setPoint, setPointMin, setPointMax, motor_id);
	mvprintw(row++, 0, "P gain:          %d       ", Pgain);
	mvprintw(row++, 0, "I gain:          %d       ", Igain);
	mvprintw(row++, 0, "D gain:          %d       ", Dgain);
	mvprintw(row++, 0, "forward gain:    %d       ", forwardGain);
	mvprintw(row++, 0, "deadband:        %d       ", deadband);
	mvprintw(row++, 0, "set point:       %d       ", setPoint);
	print(row++, 0, cols, "-");
//	mvprintw(21, 0, "polyPar: %.5f  %.5f  %.5f  %.5f    ", myoControl->polyPar[motor_id][0],
//			myoControl->polyPar[motor_id][1], myoControl->polyPar[motor_id][2],
//			myoControl->polyPar[motor_id][3]);
	mvprintw(row++, 0, "set point limits: %d to %d     ", setPointMin, setPointMax);
	mvprintw(row++, 0, "weight: %.2f     ", myoControl->getWeight());
	mvprintw(row++, 0, "SPI %s               ", (myoControl->getSPIactive(motor_id)?"active":"inactive"));
	mvprintw(row++, 0, "control_mode %d      ", myoControl->getControlMode(motor_id));
	refresh();
}

void Interface::processing(char *msg1, char *what, char *msg2) {
	char cmd;
	uint a = strlen(msg1);
	uint b = strlen(what);
	uint c = strlen(msg2);

	print(6, 0, cols, " ");
	printMessage(6, 0, msg1);
	printMessage(6, a + 1, what);
	printMessage(6, a + 1 + b + 1, msg2);
	mvchgat(6, 0, a + 1 + b, A_BLINK, 2, NULL);
	mvchgat(6, a + 1 + b + 1, a + 1 + b + 1 + c, A_BLINK, 1, NULL);
	timeout(timeout_ms);
	do {
		querySensoryData();
		cmd = mvgetch(6, a + 1 + b + 1 + c);
	} while (cmd != 'q');
	timeout(-1);
}

void Interface::processing(char *msg1, char *msg2) {
	char cmd;
	uint a = strlen(msg1);
	uint c = strlen(msg2);

	print(6, 0, cols, " ");
	printMessage(6, 0, msg1);
	printMessage(6, a + 1, msg2);
	mvchgat(6, 0, a, A_BLINK, 2, NULL);
	mvchgat(6, a + 1, a + 1 + c, A_BLINK, 1, NULL);
	timeout(timeout_ms);
	do {
		querySensoryData();
		cmd = mvgetch(6, a + 1 + c);
	} while (cmd != 'q');
	timeout(-1);
}

void Interface::toggleSPI(){
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	bool spi_active = myoControl->toggleSPI();
	if(spi_active)
		printMessage(5, 0, "SPI active", GREEN);
	else
		printMessage(5, 0, "SPI inactive", RED);
	refresh();
	usleep(1000*100);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
}

void Interface::reset(){
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	myoControl->reset();
	printMessage(5, 0, "myo control reset", GREEN);
	refresh();
	usleep(1000*100);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
}

void Interface::setGains(){
	timeout(-1);
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	printMessage(5, 0, choosecontrolstring);
	mvchgat(5, 0, strlen(choosecontrolstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	int mode = atoi(inputstring);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	printMessage(5, 0, kpstring);
	mvchgat(5, 0, strlen(kpstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	print(6, 0, cols, " ");
	uint16_t Kp = atoi(inputstring);
	printMessage(5, 0, kistring);
	mvchgat(5, 0, strlen(kistring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	print(6, 0, cols, " ");
	uint16_t Ki = atoi(inputstring);
	printMessage(5, 0, kdstring);
	mvchgat(5, 0, strlen(kdstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	print(6, 0, cols, " ");
	uint16_t Kd = atoi(inputstring);
	myoControl->setPIDcontrollerParams(Kp,Kd,Ki,0,0,motor_id,mode);
	noecho();
}

void Interface::positionControl() {
	timeout(-1);
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	myoControl->changeControl(motor_id, POSITION);
	printMessage(5, 0, setpositionstring);
	mvchgat(5, 0, strlen(setpositionstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	pos = atoi(inputstring);
	myoControl->setPosition(motor_id, pos);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	noecho();
}

void Interface::velocityControl() {
	timeout(-1);
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	myoControl->changeControl(motor_id, VELOCITY);
	printMessage(5, 0, setvelstring);
	mvchgat(5, 0, strlen(setvelstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	pos = atoi(inputstring);
	myoControl->setVelocity(motor_id, pos);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	noecho();
}

void Interface::displacementControl() {
	timeout(-1);
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	myoControl->changeControl(motor_id, DISPLACEMENT);
	printMessage(5, 0, setdisplacementstring);
	mvchgat(5, 0, strlen(setdisplacementstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	pos = atof(inputstring);
	myoControl->setDisplacement(motor_id, pos);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	noecho();
}

void Interface::switchMotor() {
	timeout(-1);
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	printMessage(5, 0, motorstring, GREEN);
	mvgetnstr(6, 0, inputstring, 30);
	uint motorrequest = atoi(inputstring);
	if (motorrequest < myoControl->numberOfMotors)
		motor_id = motorrequest;
	else {
		print(5, 0, cols, " ");
		print(6, 0, cols, " ");
		printMessage(6, 0, invalidstring, RED);
		return;
	}
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	noecho();
}

void Interface::zeroWeight(){
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	printMessage(4, 0, "zeroing weight");
	myoControl->zeroWeight();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
}

void Interface::setAllTo() {
	timeout(-1);
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	printMessage(5, 0, choosecontrolstring);
	mvchgat(5, 0, strlen(choosecontrolstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(6, 0, inputstring, 30);
	pos = atoi(inputstring);
	switch(pos){
	case 0:
		printMessage(6, 0, setpositionstring);
		mvchgat(5, 0, strlen(setpositionstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(6, 0, inputstring, 30);
		pos = atoi(inputstring);
		myoControl->allToPosition(pos);
		break;
	case 1:
		printMessage(6, 0, setvelstring);
		mvchgat(5, 0, strlen(setvelstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(6, 0, inputstring, 30);
		pos = atoi(inputstring);
		myoControl->allToVelocity(pos);
		break;
	case 2:
		printMessage(6, 0, setpositionstring);
		mvchgat(5, 0, strlen(setdisplacementstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(6, 0, inputstring, 30);
		pos = atoi(inputstring);
		myoControl->allToDisplacement(pos);
		break;
	default:
		print(5, 0, cols, " ");
		print(6, 0, cols, " ");
		printMessage(6, 0, invalidstring, RED);
		break;
	}
	usleep(10000);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	noecho();
}

void Interface::estimateSpringParameters(){
	echo();
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
	printMessage(5, 0, "estimating spring paramters, please wait...");
	myoControl->estimateSpringParameters(motor_id, 600000, 1000);
	print(5, 0, cols, " ");
	print(6, 0, cols, " ");
}

void Interface::recordTrajectories() {
   timeout(-1);
   echo();
   print(4, 0, cols, " ");
   print(5, 0, cols, " ");
   printMessage(4, 0, filenamestring);
   mvgetnstr(4, strlen(filenamestring), inputstring, 30);
   std::string name(inputstring);
   print(4, 0, cols, " ");
   printMessage(4, 0, samplingtimestring, CYAN);
   mvgetnstr(4, strlen(samplingtimestring), inputstring, 30);
   float samplingTime = atof(inputstring);
   printMessage(5, 0, recordtimestring, CYAN);
   mvgetnstr(5, strlen(recordtimestring), inputstring, 30);
   double recordTime = atof(inputstring);
   print(4, 0, cols, " ");
   print(5, 0, cols, " ");
   printMessage(4, 0, recordingstring, RED);
   map<int,vector<float>> trajectories;
   vector<int> idList = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
   vector<int> controlmode(16, 1);
   float averageSamplingTime = myoControl->recordTrajectories(samplingTime, recordTime, trajectories, idList,
														  controlmode, name);
   print(4, 0, cols, " ");
   printMessage(4, 0, donestring, GREEN);
   char averagetimestring[50];
   sprintf(averagetimestring, "average %s%f", samplingtimestring, averageSamplingTime);
   printMessage(4, strlen(donestring), averagetimestring, CYAN);
   usleep(500000);
   print(4, 0, cols, " ");
   print(5, 0, cols, " ");
}

void Interface::playTrajectories(){
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	printMessage(4, 0, filenamestring);
	mvgetnstr(4, strlen(filenamestring), inputstring, 30);
	myoControl->playTrajectory(inputstring);
	print(4, 0, cols, " ");
	printMessage(4, 0, donestring, GREEN);
	usleep(500000);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
}
