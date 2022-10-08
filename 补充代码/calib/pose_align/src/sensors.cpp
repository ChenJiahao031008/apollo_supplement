#include "pose_align/sensors.h"

namespace pose_align {

    OdomTformData::OdomTformData(Timestamp timestamp, Transform T_o0_ot)
            : timestamp_(timestamp), T_o0_ot_(T_o0_ot) {}

    Transform &OdomTformData::getTransform() { return T_o0_ot_; }

    Timestamp &OdomTformData::getTimestamp() { return timestamp_; }

    void Odom::addTransformData(Timestamp &timestamp, Transform &T) {
        data_.emplace_back(timestamp, T);
    }

    Transform Odom::getOdomTransform(Timestamp timestamp,
                                     size_t start_idx,
                                     size_t *match_idx) {
        size_t idx = start_idx;

//        std::cout<<"data_ size is "<<data_.size()<<std::endl;

        while ((idx < (data_.size() - 1)) &&
               (timestamp > data_[idx].getTimestamp())) {
            ++idx;
        }
        if (idx > 0) {
            --idx;
        }

        if (match_idx != nullptr) {
            *match_idx = idx;
        }

        // interpolate
        double t_diff_ratio =
                static_cast<double>(timestamp - data_[idx].getTimestamp()) /
                static_cast<double>(data_[idx + 1].getTimestamp() -
                                    data_[idx].getTimestamp());

        Transform::Vector6 diff_vector =
                (data_[idx].getTransform().inverse() * data_[idx + 1].getTransform())
                        .log();
        Transform out =
                data_[idx].getTransform() * Transform::exp(t_diff_ratio * diff_vector);

        // Transform data_idx = data_[idx].getTransform();

        // Eigen::Quaterniond pre_q = data_[idx].getTransform().rotation_;
        // Eigen::Quaterniond cur_q = data_[idx + 1].getTransform().rotation_;

        // Eigen::Quaterniond delta_q = pre_q.slerp(t_diff_ratio, cur_q);
        // Eigen::Vector3d delta_t = t_diff_ratio * (data_[idx].getTransform().translation_ - 
        //                                           data_[idx + 1].getTransform().translation_);

        // Transform delta_T = Transform(delta_t, delta_q);

        // Transform out = data_[idx].getTransform();
        // // out.translation_ = (out.translation_ + delta_t).eval();
        // // out.rotation_ = out.rotation_ * delta_q;

        return out;
    }

}  // namespace pose_align
