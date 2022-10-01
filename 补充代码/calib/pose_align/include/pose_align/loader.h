#pragma once

#include <string>
#include "pose_align/sensors.h"

namespace pose_align {

    class Loader {
    public:
        Loader();

        bool loadPoses(std::string &data_path);

        Odom odom_source;
        Odom odom_target;
    };
}  // namespace pose_align

