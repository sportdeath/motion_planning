A lightweight implementation of the RRT\* algorithm.

## Installation

Clone this library with git submodules:

    git clone --recurse-submodules git@github.mit.edu:tfh/motion_planning.git

Or add the submodules after cloning

    git clone --recurse-submodules git@github.mit.edu:tfh/motion_planning.git
    git submodule init
    git submodule update

## Dependencies

This library requires the [libpng](http://www.libpng.org/pub/png/libpng.html) library.
On Ubuntu it can be installed with:

    sudo apt-get install libpng-dev

On Arch Linux it can be installed with:

    sudo pacman -S libpng

Or to install manually:

    git clone git://git.code.sf.net/p/libpng/code libpng
    cd libpng
    ./autogen.sh
    ./configure --prefix=[INSTALL_PATH]
    make check
    make install

## Building

    cd motion_planning
    mkdir build
    cd build
    cmake ..
    make

## Usage

In order to compute RRT\*, you must first describe the vehicle dynamics, the map, and the sampling function.

### Vehicle Dynamics

You must define vehicle dynamics that inherit from the ```Steer``` class, which describes the shortest path between any two points in free space.
For a robot that can move in any direction these paths would just be straight lines.
For more constrained robots, like cars, these paths become more complicated.

This library comes with ```ReedsSheepSteer```, which models a simple car that can drive forwards and backwards along straight lines and cricles.
It can be defined with the minimum turning radius. For example if the car is able to turn around a ```0.3m``` circle, you would do:

    ReedsSheepSteer steer(0.3)

### Map

You must define a map that inherits from the ```Occupancy``` class.
This library comes with ```OccupancyGrid2D``` which is a standard 2D occupancy grid.
These maps can be built in a variety of ways, for example with a vector of occupancy values

    OccupancyGrid2D<Pose2D> occ;
    size_t height = 100;
    size_t width = 100;
    std::vector<double> occupancyValues(height * width, 0.) // A vector of HxW zeros
    double mapResolution = 0.05;
    Pose2D mapOrigin = {.x=0, .y=0, .theta=0}; // The coordinates of the top left cell
    occ.setMap(occupancyValues, width, height, mapResolution, mapOrigin);

Or frome a grayscale PNG:

    ...
    occ.setMap("maps/tiny.png", mapResolution, mapOrigin);


### Sampling Function

Because RRT\* is a sample-based algorithm, you must define the function used to sample states on the map.
This library comes with a variety of sampling algorithms like uniform sampling, gaussian sampling, and bridge sampling (useful for narrow passages). These sampling algorithms all inherit from the ```StateSampler``` class.
    
    UniformSampler<Pose2D> uni(&occ); 
    BridgeSampler brg(&occ, 0.1); // 0.1 defines the stddev of the bridge
    
    // Use uniform with probability 0.3 and
    // bridge with probability 0.7
    std::vector<StateSampler<Pose2D> *> samplers = {&uni, &brg};
    std::vector<double> sampler_weights = {3., 7.};
    MixedSampler<Pose2D> sampler(samplers, weights);

### Computing RRT\*

Once the dynamics, map, and sampling are computed they can by used to construct RRT\*.
In addition we will need the origin of the search, and the search radius which describes
the distance at which points are connected.

    Pose2d rrtOrigin = {.x=3, .y=4, .theta=0.5};
    double rrt
    RRTStar<Pose2D> rrt(
        &steer,
        &occ,
        sampler.sampleFunction(),
        rrtOrigin,
        rrtRadius);
        
Once RRT\* has been constructed, you can iterate by running something like:

    int numNodes = 0;
    for (int i = 0; i < 1000; i++) {
        if (rrt.iterate()) numNodes++;
    }

There are a variety of ways to interact with the resulting tree.
For example you could get the root and use it in its tree structure:

    const RRTStar<Pose2D>::Node & root = rrt.root();

    // Look at the state of the node
    Pose2D nodeState = node.state;

    // Or look at its parent
    // (the root's parent is NULL)
    RRTStar<Pose2D>::Node * nodeParent = node.parent;

    // Or look at its children
    std::vector<Node *> children = node.parent;
    
Or you could iterate over all the nodes in the tree:

    for (const RRTStar<Pose2D>::Node & node : rrt.getNodes()) {
        ...
    }

Or you could sample it as a list of states:

    // The path from the root to someNode
    // sampled every 0.1 meters
    std::vector<Pose2D> path = rrt.samplePath(&someNode, 0.1);
