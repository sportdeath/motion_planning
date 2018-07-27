#ifndef DUBINS_STEER_HPP
#define DUBINS_STEER_HPP

extern "C" {
#include "dubins.h"
}

#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/State/Pose2D.hpp"

class DubinsSteer : public Steer<Pose2D> {
private:
    double turningRadius;

    DubinsPath path;

    struct DubinsSampleData {
        std::vector<Pose2D> * samples;
        size_t sampleIndex;
    };
    static int dubinsSampleCallback(double q[3], double x, void * user_data);

public:
    DubinsSteer(double turningRadius_) : turningRadius(turningRadius_) {};

    bool steer(const Pose2D * start, const Pose2D * end);

    std::vector<Pose2D> sample(double resolution);

    double cost();

    double lowerBoundCost(const Pose2D * state, const Pose2D * end) const;
};

#endif // DUBINS_STEER_HPP
