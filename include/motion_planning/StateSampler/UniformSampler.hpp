template <class State>
class UniformSampler {
public:
    UniformSampler(Occupancy<State> * occupancy);
    virtual State sample();
};
