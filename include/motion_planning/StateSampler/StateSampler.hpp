#include <functional>

#include <motion_planning/Occupancy/Occupancy.hpp>

template<class State>
class StateSampler {
public:
    virtual State sample() const = 0;

    std::function<Pose2D(void)> sampleFunction() const {
        return std::bind(&StateSampler<State>::sample, this);
    }
};

template<class State>
class UniformSampler : public StateSampler<State> {
private:
    Occupancy<State> * occupancy;
    const bool sampleFree;
    const bool sampleUnknown;
    const bool sampleOccupied;

public:
    UniformSampler(
        Occupancy<State> * occupancy_, 
        bool sampleFree_, 
        bool sampleUnknown_, 
        bool sampleOccupied_);

    State sample() const;
};

class BridgeSampler : public StateSampler<Pose2D> {
private:
    Occupancy<Pose2D> * occupancy;
    UniformSampler<Pose2D> sampleOccupied;
    
public:
    BridgeSampler(Occupancy<Pose2D> * occupancy_);

    Pose2D sample() const;
};

template<class State>
class MixedSampler : public StateSampler<State> {
private:
    std::vector<StateSampler<State> *> samplers;
    std::vector<double> cumulativeProbabilities;

public:
    MixedSampler(
        std::vector<StateSampler<State> *> & samplers_,
        std::vector<double> & weights);

    State sample() const;
};
