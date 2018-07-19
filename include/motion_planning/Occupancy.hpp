template <class State>
class Occupancy {
public:
    virtual bool isFree(State const) = 0;
};
