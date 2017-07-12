#include "myoControl.hpp"

MyoControl::MyoControl(vector<int32_t*> &myo_base):myo_base(myo_base){
	// initialize control mode
	numberOfMotors = myo_base.size()*MOTORS_PER_MYOCONTROL;
	// initialize all controllers with default values
	control_Parameters_t params;
	getDefaultControlParams(&params, POSITION);
	for(uint motor=0;motor<numberOfMotors;motor++){
		changeControl(motor, 0, params);
		control_params[motor][POSITION] = params;
	}
	getDefaultControlParams(&params, VELOCITY);
	for(uint motor=0;motor<numberOfMotors;motor++){
		control_params[motor][VELOCITY] = params;
	}
	getDefaultControlParams(&params, DISPLACEMENT);
	for(uint motor=0;motor<numberOfMotors;motor++){
		control_params[motor][DISPLACEMENT] = params;
	}
}

MyoControl::~MyoControl(){
	cout << "shutting down myoControl" << endl;
}

void MyoControl::changeControl(int motor, int mode, control_Parameters_t &params){
	int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
	MYO_WRITE_reset_controller(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset);
	MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.Kp);
	MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.Kd);
	MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.Ki);
	MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.forwardGain);
	MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.deadBand);
	MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.IntegralPosMax);
	MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.IntegralNegMax);
	MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.outputPosMax);
	MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.outputNegMax);
}

void MyoControl::changeControl(int motor, int mode){
	int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
	MYO_WRITE_reset_controller(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset);
	MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].Kp);
	MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].Kd);
	MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, control_params[motor][mode].Ki);
	MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].forwardGain);
	MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, (control_params[motor][mode].deadBand));
	MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].IntegralPosMax);
	MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].IntegralNegMax);
	MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].outputPosMax);
	MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].outputNegMax);
	MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
}

void MyoControl::changeControl(int mode){
	for(uint motor=0;motor<numberOfMotors;motor++){
		int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
		MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
		MYO_WRITE_reset_controller(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset);
		MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].Kp);
		MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].Kd);
		MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].Ki);
		MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].forwardGain);
		MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, (control_params[motor][mode].deadBand));
		MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].IntegralPosMax);
		MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].IntegralNegMax);
		MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].outputPosMax);
		MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].outputNegMax);
		MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
	}
}

bool MyoControl::toggleSPI(){
	bool spi_active;
	for(uint i=0;i<myo_base.size();i++){
		spi_active = MYO_READ_spi_activated(myo_base[i]);
		MYO_WRITE_spi_activated(myo_base[i],!spi_active);
	}

	return !spi_active;
}

void MyoControl::reset(){
	for(uint i=0;i<myo_base.size();i++){
		MYO_WRITE_reset_myo_control(myo_base[i],true);
		MYO_WRITE_reset_myo_control(myo_base[i],false);
	}
}

void MyoControl::setPosition(int motor, int32_t position){
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0), position);
}

void MyoControl::setVelocity(int motor, int16_t velocity){
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0), velocity);
}

void MyoControl::setDisplacement(int motor, int16_t displacement){
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0), displacement);
}

bool MyoControl::setSPIactive(int motor, bool active){
	MYO_WRITE_spi_activated(myo_base[motor/MOTORS_PER_MYOCONTROL], active);
}

void MyoControl::getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
									int &setPoint, int &setPointMin, int &setPointMax, int motor){
	Pgain = MYO_READ_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
	Igain = MYO_READ_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
	Dgain = MYO_READ_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
	forwardGain = MYO_READ_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
	deadband = MYO_READ_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
	setPoint = MYO_READ_sp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
	setPointMin = 0;
	setPointMax = 0;
}

void MyoControl::setPIDcontrollerParams(uint16_t Pgain, uint16_t Igain, uint16_t Dgain, uint16_t forwardGain, uint16_t deadband, int motor, int mode){
	control_params[motor][mode].Kp = Pgain;
	control_params[motor][mode].Ki = Igain;
	control_params[motor][mode].Kd = Dgain;
	control_params[motor][mode].forwardGain = forwardGain;
	control_params[motor][mode].deadBand = deadband;
}

uint16_t MyoControl::getControlMode(int motor){
	return MYO_READ_control(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
}

int16_t MyoControl::getPWM(int motor){
	return MYO_READ_pwmRef(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
}

int32_t MyoControl::getPosition(int motor){
	return MYO_READ_position(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
}

int16_t MyoControl::getVelocity(int motor){
	return MYO_READ_velocity(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
}

int16_t MyoControl::getDisplacement(int motor){
	return MYO_READ_displacement(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
}

int16_t MyoControl::getCurrent(int motor){
	return MYO_READ_current(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0));
}

bool MyoControl::getSPIactive(int motor){
	return MYO_READ_spi_activated(myo_base[motor/MOTORS_PER_MYOCONTROL]);
}

void MyoControl::getDefaultControlParams(control_Parameters_t *params, int control_mode){
	params->outputPosMax = 1000;
	params->outputNegMax = -1000;

	params->radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);

switch(control_mode){
case POSITION:
	params->spPosMax = 10000000;
	params->spNegMax = -10000000;
	params->Kp = 1.0;
	params->Ki = 0;
	params->Kd = 0;
	params->forwardGain = 0;
	params->deadBand = 0;
	params->IntegralPosMax = 100;
	params->IntegralNegMax = -100;
	break;
case VELOCITY:
	params->spPosMax = 100;
	params->spNegMax = -100;
	params->Kp = 1.0;
	params->Ki = 0;
	params->Kd = 0;
	params->forwardGain = 0;
	params->deadBand = 0;
	params->IntegralPosMax = 100;
	params->IntegralNegMax = -100;
	break;
case DISPLACEMENT:
	params->spPosMax = 2000;
	params->spNegMax = 0;
	params->Kp = 1.0;
	params->Ki = 0;
	params->Kd = 0;
	params->forwardGain = 0;
	params->deadBand = 0;
	params->IntegralPosMax = 100;
	params->IntegralNegMax = 0;
	break;
default:
	cout << "unknown control mode" << endl;
	break;
}

}

void MyoControl::allToPosition(int32_t pos){
	changeControl(POSITION);
	for(uint motor=0; motor<numberOfMotors; motor++){
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0), pos);
	}
}

void MyoControl::allToVelocity(int16_t vel){
	changeControl(VELOCITY);
	for(uint motor=0; motor<numberOfMotors; motor++){
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0), vel);
	}
}

void MyoControl::allToDisplacement(int16_t displacement){
	changeControl(DISPLACEMENT);
	for(uint motor=0; motor<numberOfMotors; motor++){
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-(motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0), displacement);
	}
}

void MyoControl::zeroWeight(){
	weight_offset = -getWeight();
}

float MyoControl::getWeight(){
	float weight = 0;
	uint32_t adc_value = 0;
	if(adc_base!=nullptr){
		*adc_base = 0;
		adc_value = *adc_base;
		weight = (adc_weight_parameters[0]+weight_offset+adc_weight_parameters[1]*adc_value);
	}
	return weight;
}

float MyoControl::recordTrajectories(
        float samplingTime, float recordTime,
		map<int,vector<float>> &trajectories, vector<int> &idList,
        vector<int> &controlmode, string name) {
    // this will be filled with the trajectories
    allToDisplacement(200);

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;

    // start recording
    timer.start();
    do{
        dt = elapsedTime;
        for (uint motor = 0; motor < idList.size(); motor++) {
			if (controlmode[motor] == POSITION)
				trajectories[idList[motor]].push_back( getPosition(motor) );
			else if (controlmode[motor] == VELOCITY)
				trajectories[idList[motor]].push_back( getVelocity(motor) );
			else if (controlmode[motor] == FORCE)
				trajectories[idList[motor]].push_back( getDisplacement(motor) );
        }
        sample++;
        elapsedTime = timer.elapsedTime();
        dt = elapsedTime - dt;
        // if faster than sampling time sleep for difference
        if (dt < samplingTime) {
            usleep((samplingTime - dt) * 1000000.0);
            elapsedTime = timer.elapsedTime();
        }
    }while(elapsedTime < recordTime);

    // set force to zero
	allToDisplacement(0);

    // done recording
    std::ofstream outfile;
    if (name.empty()) {
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char str[200];
        sprintf(str, "recording_%s.log",
                asctime(timeinfo));
        name = str;
    }

    outfile.open(name);
    if (outfile.is_open()) {
        outfile << "<?xml version=\"1.0\" ?>"
                << std::endl;
        uint m = 0;
        char motorname[10];
        for (uint m = 0; m < idList.size(); m++) {
            sprintf(motorname, "motor%d", idList[m]);
            outfile << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\""
                    << controlmode[m] << "\" samplingTime=\"" << samplingTime*1000.0f << "\">"
                    << std::endl;
            outfile << "<waypointlist>" << std::endl;
            for (uint i = 0; i < trajectories[idList[m]].size(); i++)
                outfile << trajectories[idList[m]][i] << " ";
            outfile << "</waypointlist>" << std::endl;
            outfile << "</trajectory>" << std::endl;
        }
        outfile << "</roboybehavior>" << std::endl;
        outfile.close();
    }

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

bool MyoControl::playTrajectory(const char* file){
    // initialize TiXmlDocument doc with a string
    TiXmlDocument doc(file);
    if (!doc.LoadFile()) {
        return false;
    }

    TiXmlElement *root = doc.RootElement();

    map<int,vector<float>> trajectories;
    int samplingTime, numberOfSamples;

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *trajectory_it = NULL;
    for (trajectory_it = root->FirstChildElement("trajectory"); trajectory_it;
    		trajectory_it = trajectory_it->NextSiblingElement("trajectory")) {
        if (trajectory_it->Attribute("motorid") && trajectory_it->QueryIntAttribute("samplingTime",&samplingTime)) {
        	int motor;
			if(trajectory_it->QueryIntAttribute("motorid", &motor) != TIXML_SUCCESS){
				return false;
			}
			TiXmlElement *waypointlist_it = trajectory_it->FirstChildElement("waypointlist");
			stringstream stream(waypointlist_it->GetText());
			while(1) {
				int n;
				stream >> n;
				trajectories[motor].push_back(n);
				if(!stream){
					numberOfSamples = trajectories.size();
					break;
				}
			}
        }
    }
    allToDisplacement(0);
    timer.start();
    double elapsedTime = 0.0, dt;
    int sample = 0;
    samplingTime /= 1000.0f;
    do{
		dt = elapsedTime;
		for (auto &motor : trajectories) {
			setDisplacement(motor.first, motor.second[sample]);
		}
		sample++;

		elapsedTime = timer.elapsedTime();
		dt = elapsedTime - dt;
		// if faster than sampling time sleep for difference
		if (dt < samplingTime) {
			usleep((samplingTime - dt) * 1000000.0);
			elapsedTime = timer.elapsedTime();
		}
    }while(timer.elapsedTime()<(numberOfSamples*samplingTime/1000.0f));

    return true;
}

void MyoControl::estimateSpringParameters(int motor, int timeout,  uint numberOfDataPoints){
	vector<float> weight, displacement, coeffs;
	setDisplacement(motor,0);
	changeControl(motor,DISPLACEMENT);
	float force_min = 0, force_max = 4.0;
	milliseconds ms_start = duration_cast< milliseconds >(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
	ofstream outfile;
	outfile.open ("springParameters_calibration.csv");
	do{
		float f = (rand()/(float)RAND_MAX)*(force_max-force_min)+force_min;
		setDisplacement(motor, f);
		t0 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		do{// wait a bit until force is applied
			// update control
			t1 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		}while((t1-t0).count()<500);

		// note the weight
		weight.push_back(getWeight());
		// note the force
		displacement.push_back(getDisplacement(motor));
		outfile << displacement.back() << ", " << weight.back() << endl;
		ms_stop = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	}while((ms_stop-ms_start).count()<timeout && weight.size()<numberOfDataPoints);
	polynomialRegression(3, displacement, weight, coeffs);
//	polyPar[motor] = coeffs;
	outfile.close();
}

void MyoControl::polynomialRegression(int degree, vector<float> &x, vector<float> &y,
			vector<float> &coeffs){
		int N = x.size(), i, j, k;
	    double X[2*degree+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	    for (i=0;i<2*degree+1;i++)
	    {
	        X[i]=0;
	        for (j=0;j<N;j++)
	            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	    }
	    double B[degree+1][degree+2],a[degree+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	    for (i=0;i<=degree;i++)
	        for (j=0;j<=degree;j++)
	            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	    double Y[degree+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
	    for (i=0;i<degree+1;i++)
	    {
	        Y[i]=0;
	        for (j=0;j<N;j++)
	        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	    }
	    for (i=0;i<=degree;i++)
	        B[i][degree+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	    degree=degree+1;                //degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	    for (i=0;i<degree;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
	        for (k=i+1;k<degree;k++)
	            if (B[i][i]<B[k][i])
	                for (j=0;j<=degree;j++)
	                {
	                    double temp=B[i][j];
	                    B[i][j]=B[k][j];
	                    B[k][j]=temp;
	                }

	    for (i=0;i<degree-1;i++)            //loop to perform the gauss elimination
	        for (k=i+1;k<degree;k++)
	            {
	                double t=B[k][i]/B[i][i];
	                for (j=0;j<=degree;j++)
	                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
	            }
	    for (i=degree-1;i>=0;i--)                //back-substitution
	    {                        //x is an array whose values correspond to the values of x,y,z..
	        a[i]=B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
	        for (j=0;j<degree;j++)
	            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
	                a[i]=a[i]-B[i][j]*a[j];
	        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	    }
	    for (i=0;i<degree;i++)
	        coeffs.push_back(a[i]);	//the values of x^0,x^1,x^2,x^3,....
}
