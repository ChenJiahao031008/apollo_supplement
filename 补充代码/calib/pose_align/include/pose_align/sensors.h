
#pragma once

#include <random>

#include "pose_align/transform.h"

typedef double Timestamp;  // the timestamp set as sec

namespace pose_align {

    class OdomTformData {
    public:
        OdomTformData(Timestamp timestamp, Transform T_o0_ot);

        Transform &getTransform();

        Timestamp &getTimestamp();

    private:
        Transform T_o0_ot_;
        Timestamp timestamp_;
    };

    class Odom {
    public:
        void addTransformData(Timestamp &timestamp,
                              Transform &transform);

        Transform getOdomTransform(Timestamp timestamp,
                                   size_t start_idx = 0,
                                   size_t *match_idx = nullptr);

        Transform getOdomTransformOfidx(size_t idx) {
            return data_[idx].getTransform();
        }

        bool empty() { return data_.empty(); }

        int getOdomTransformSize() {
            return data_.size();
        }

    private:
        std::vector<OdomTformData> data_;
    };

}  // namespace pose_align
