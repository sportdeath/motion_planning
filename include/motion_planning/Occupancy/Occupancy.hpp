template <class State>
class Occupancy {
public:
    virtual bool isFree(const State &state) = 0;
    virtual bool isFree(Steer &steer) = 0;
};
