#include <vector>
#include <limits>

class DistanceTransform {
private:
    std::vector<int> parabolaIdxs;
    std::vector<double> parabolaBoundaries;
    static constexpr double inf = std::numeric_limits<double>::infinity();
    
public:
    DistanceTransform(int maxSize);

    void distanceSquared1D(const std::vector<double> & input, std::vector<double> & output);

    void distanceSquared2D(std::vector<double> & input, int width, int height);

    void distance2D(std::vector<double> & input, int width, int height);
};
