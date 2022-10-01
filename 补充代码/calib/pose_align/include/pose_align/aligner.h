#pragma once 

#include <ceres/ceres.h>

#include <future>
#include <limits>
#include <omp.h>
#include "pose_align/sensors.h"

namespace pose_align {

    class Aligner {
    public:
        Aligner() {}
        void align(Odom &odom_source, Odom &odom_target);
        bool fix_extrinsic_z_ = true;
    };

}  // namespace pose_align

