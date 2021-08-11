#include <fstream>
#include <iostream>
#include "measurement_package.h"
#include <sstream>
#include <vector>
#include "../Eigen/Dense"
#include "tracking.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

int main() {
    
    vector<MeasurementPackage> measurement_pack_list;

    // hardcoded input file with laser and radar measurements
	string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

    if (!in_file.is_open()) {
        cout << "Cannot open this file: " << in_file_name_ <<endl;
    }
    
    string line;
    int i = 0;
    while (getline(in_file,line)&&(i<=3))
    {

        MeasurementPackage measurement_package;
        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;
        int64_t timestamp;
        if(sensor_type.compare("L") == 0){
            //read measurements
            measurement_package.sensor_type_ = MeasurementPackage::LASER;
            measurement_package.raw_measurement_ = VectorXd(2);
            float x, y;
            iss >> x;
            iss >> y;
            measurement_package.raw_measurement_ << x , y;
            iss >> timestamp;
            measurement_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(measurement_package);
        }
        else if(sensor_type.compare("R") == 0)
            continue;

        i++;
        
    }
    
    //call tracking instance
    Tracking tracking;

    size_t num_measurement_pack = measurement_pack_list.size();
    for(size_t i = 0; i < num_measurement_pack; i++){
        tracking.ProcessMeasurement(measurement_pack_list[i]);
    }
    
    if(in_file.is_open()){
        in_file.close();
    }
    
    return 0;
}