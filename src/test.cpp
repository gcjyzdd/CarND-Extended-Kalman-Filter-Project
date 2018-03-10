#include <iostream>
#include <sstream>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

int main()
{
	  // Create a Kalman Filter instance
	  FusionEKF fusionEKF;

	  vector<MeasurementPackage> measurement_pack_list;

	  	// hardcoded input file with laser and radar measurements
	  	string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
	  	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

	  	if (!in_file.is_open()) {
	  		cout << "Cannot open input file: " << in_file_name_ << endl;
	  	}

	  	string line;
	  	// set i to get only first 3 measurments
	  	int i = 0;
	  	while(getline(in_file, line) && (i<=30)){

	  		MeasurementPackage meas_package;

	  		istringstream iss(line);
	  		string sensor_type;
	  		iss >> sensor_type;	//reads first element from the current line
	  		int64_t timestamp;
	  		if(sensor_type.compare("L") == 0){	//laser measurement
	  			//read measurements
	  			meas_package.sensor_type_ = MeasurementPackage::LASER;
	  			meas_package.raw_measurements_ = VectorXd(2);
	  			float x;
	  			float y;
	  			iss >> x;
	  			iss >> y;
	  			meas_package.raw_measurements_ << x,y;
	  			iss >> timestamp;
	  			meas_package.timestamp_ = timestamp;
	  			measurement_pack_list.push_back(meas_package);

	  		}else if(sensor_type.compare("R") == 0){
	  			//Skip Radar measurements
	  			continue;
	  		}
	  		i++;

	  	}

	  return 0;
}
