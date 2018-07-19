extern "C" {
#include "dubins.h"
}

#include "motion_planning/Steer.hpp"

struct Pose2D {
    double x;
    double y;
    double theta;
};

class DubinsSteer : public Steer<Pose2D> {
private:
    DubinsPath path;

    struct DubinsSampleData {
        std::vector<Pose2D> * samples;
        int sampleIndex;
    };
    static int dubinsSampleCallback(double q[3], double x, void * user_data);

public:
    DubinsSteer(Pose2D start, Pose2D end, double turningRadius);

    std::vector<Pose2D> sample(double resolution);

    double cost();
};
