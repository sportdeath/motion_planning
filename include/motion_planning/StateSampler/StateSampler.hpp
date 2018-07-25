#include <functional>
#include <random>

#include <motion_planning/Occupancy/Occupancy.hpp>

template<class State>
class StateSampler {
public:
    virtual State sample() = 0;

    std::function<Pose2D(void)> sampleFunction() {
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
        bool sampleFree_=true, 
        bool sampleUnknown_=false, 
        bool sampleOccupied_=false);

    State sample();
};

class BridgeSampler : public StateSampler<Pose2D> {
private:
    Occupancy<Pose2D> * occupancy;
    UniformSampler<Pose2D> sampleOccupied;
    std::default_random_engine generator;
    std::normal_distribution<double> normalDistribution;
    
public:
    BridgeSampler(Occupancy<Pose2D> * occupancy_, double stdDev);

    Pose2D sample();
};

template<class State>
class MixedSampler : public StateSampler<State> {
private:
    std::vector<StateSampler<State> *> samplers;
    std::vector<double> cumulativeProbabilities;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> choiceDistribution;

public:
    MixedSampler(
        std::vector<StateSampler<State> *> & samplers_,
        std::vector<double> & weights);

    State sample();
};
