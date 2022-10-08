#include "pose_align/aligner.h"
#include "pose_align/loader.h"

#include <string>

using namespace pose_align;

void lidarToIMUCalibration(std::string& data_path)
{
    Loader loader = Loader();
    loader.loadPoses(data_path);

    Aligner aligner = Aligner();
    aligner.align(loader.odom_source, loader.odom_target);
}

int main(int argc, char ** argv){
    std::string data_path = argv[1];
    lidarToIMUCalibration(data_path);
    return 0;
}