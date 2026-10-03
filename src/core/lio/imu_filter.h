#pragma once

#include <cmath>
#include <stdexcept>

#include "common/eigen_types.h"

namespace lightning {

class LowPass2 {
   public:
    void Configure(double sample_hz, double cutoff_hz) {
        if (!std::isfinite(sample_hz) || !std::isfinite(cutoff_hz) || sample_hz <= 0 ||
            cutoff_hz <= 0 || cutoff_hz >= sample_hz * 0.5) {
            throw std::invalid_argument("IMU low-pass requires 0 < cutoff_hz < sample_hz / 2");
        }

        const double k = std::tan(std::acos(-1.0) * cutoff_hz / sample_hz);
        const double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k * k);
        b0_ = k * k * norm;
        b1_ = 2.0 * b0_;
        b2_ = b0_;
        a1_ = 2.0 * (k * k - 1.0) * norm;
        a2_ = (1.0 - std::sqrt(2.0) * k + k * k) * norm;
        initialized_ = false;
    }

    void Reset(const Vec3d& value) {
        x1_ = x2_ = y1_ = y2_ = value;
        initialized_ = true;
    }

    Vec3d Filter(const Vec3d& value) {
        if (!initialized_) {
            Reset(value);
            return value;
        }

        Vec3d result = b0_ * value + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_;
        x1_ = value;
        y2_ = y1_;
        y1_ = result;
        return result;
    }

   private:
    double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
    Vec3d x1_ = Vec3d::Zero(), x2_ = Vec3d::Zero();
    Vec3d y1_ = Vec3d::Zero(), y2_ = Vec3d::Zero();
    bool initialized_ = false;
};

}  // namespace lightning
