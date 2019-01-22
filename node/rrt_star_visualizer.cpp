#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <motion_planning/StateSampler/StateSampler.hpp>
#include <motion_planning/Steer/ReedsSheppSteer.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/RRTStar.hpp>
#include <motion_planning/ros_utils.hpp>

class RRTStarVisualizer {

private:
    // The origin of the rrt search
    Pose2D start = {.x=0., .y=0., .theta=0.};

    // The ccupancy grid
    OccupancyGrid2D<Pose2D> occupancy;

    // The steering function
    ReedsSheppSteer steer;

    // A variety of sampling functions
    UniformSampler<Pose2D> freeSampler;
    BridgeSampler bridgeSampler;
    MixedSampler<Pose2D> sampler;

    // A ROS node
    ros::NodeHandle n;

    // Visualization publishers
    ros::Publisher samplePub;
    ros::Publisher pathPub;
    ros::Publisher treePub;

    // Rviz subscribers
    ros::Subscriber clickSub;
    ros::Subscriber startSub;
    ros::Subscriber endSub;
    ros::Subscriber mapSub;
    
public:
    RRTStarVisualizer() : 
        steer(1),
        freeSampler(&occupancy),
        bridgeSampler(&occupancy, 0.5)
    {
        // USE ROS PARAMETERS
        // poseArrayToRos
        // rosToPose
        // pathToRos
        // treeToRos

        // Construct the sampling function
        std::vector<StateSampler<Pose2D> *> samplers = {&freeSampler, &bridgeSampler};
        std::vector<double> weights = {3., 1.};
        sampler = MixedSampler<Pose2D>(samplers, weights);

        // Initialize occupancy with large empty map
        int height = 100;
        int width = 100;
        std::vector<int8_t> mapData(height * width, 0);
        Pose2D origin = {.x=-5., .y=-5., .theta=0.};
        occupancy.setObjectRadius(0.44, 0.02);
        occupancy.setMap(mapData, width, height, 0.1, origin);

        // Construct publishers for the visualization topics 
        samplePub = n.advertise<geometry_msgs::PoseArray>("/rrtstar/samples", 1);
        pathPub = n.advertise<visualization_msgs::Marker>("/rrtstar/path", 1);
        treePub = n.advertise<visualization_msgs::Marker>("/rrtstar/tree", 1);

        // Construct subscribers for the rviz topics
        clickSub = n.subscribe("/clicked_point", 1, &RRTStarVisualizer::clickCallback, this);
        startSub = n.subscribe("/initialpose", 1, &RRTStarVisualizer::startCallback, this);
        endSub = n.subscribe("/move_base_simple/goal", 1, &RRTStarVisualizer::endCallback, this);
        mapSub = n.subscribe("/map", 1, &RRTStarVisualizer::mapCallback, this);
    }
    
    /**
     * Fetch the occupancy grid.
     */
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg) {
        // Visualize the occupancy map
        occupancy.setMap(
            msg -> data, 
            msg -> info.width, 
            msg -> info.height, 
            msg -> info.resolution, 
            RosUtils::rosToPose(msg -> info.origin));

        std::cout << "Map received"  << std::endl;
    }

    /**
     * Visualize the distribution of sampled poses.
     */
    void clickCallback(const geometry_msgs::PointStamped::ConstPtr & msg) {
        // Sample 1000 poses
        std::vector<Pose2D> poses;
        for (int i = 0; i < 1000; i++) {
            poses.push_back(sampler.sample());
        }

        // Convert them to ROS and publish
        samplePub.publish(RosUtils::poseArrayToRos(poses));

        // The message is unused
        (void)msg;
    }


    /**
     * Visualize the growth of the RRT* tree.
     */
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
        // Convert the ros message to a pose
        start = RosUtils::rosToPose(msg -> pose.pose);

        // Initialize the RRT tree
        RRTStar<Pose2D> rrt(
            &steer,
            &occupancy,
            sampler.sampleFunction(),
            start,
            4.);

        // Perform 4000 iterations
        for (int i = 0; i <= 4000; i++) {
            rrt.iterate();
            // Every 100 iterations visualize the tree
            if (i % 100 == 0) {
                std::vector<std::vector<Pose2D>> tree = rrt.sampleTree(0.1);
                treePub.publish(RosUtils::treeToRos(tree));
            }
        }

        // Compute the average path length
        // to any node in the tree
        double averagePathLength = 0;
        for (const RRTStar<Pose2D>::Node & node : rrt.getNodes()) {
            averagePathLength += node.cost/(double)rrt.getNodes().size();
        }
        std::cout << "Number of nodes: " << rrt.getNodes().size() << std::endl;
        std::cout << "Average path length to node: " << averagePathLength << std::endl;
    }

    /**
     * Visualize the steering function.
     */
    void endCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
        // Convert the ros message to a pose
        Pose2D end = RosUtils::rosToPose(msg -> pose);

        // Initialize the steer between the two
        steer.steer(&start, &end);

        // Print the cost
        std::cout << "Steer cost : " << steer.cost() << std::endl;

        // Sample along the path and display it
        std::vector<Pose2D> path = steer.sample(0.1);
        pathPub.publish(RosUtils::pathToRos(path));
    }

};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "rrt_star_visualizer");
  RRTStarVisualizer rrt;
  ros::spin();
  return 0;
}
