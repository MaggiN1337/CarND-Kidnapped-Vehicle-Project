/**
 * test.cpp
 * used for testing methods
 */

#include <iostream>
#include <random> // Need this for sampling from distributions
#include "particle_filter.h"

using std::normal_distribution;

int main() {

    // Landmark measurement uncertainty [x [m], y [m]]
    double std [3] = {0.3, 0.3, 0.4};

    // Create particle filter
    ParticleFilter pf;
    pf.init(1,1,1,std);

    double std_2[3] = {0.5, 0.5, 0.5};
    pf.prediction(1,std_2,1,0.1);

    std::vector<LandmarkObs> pred{{1, 0.5, 0.5},{2, 1.6, 0.5},{3, 2.5, 0.9}};
    std::vector<LandmarkObs> observ{{1, 0.5, 0.6},{2, 2.5, 0.8},{3, 1.6, 0.5}};
    pf.dataAssociation(pred, observ);

    // Set up parameters here
    double delta_t = 0.1;  // Time elapsed between measurements [sec]
    double sensor_range = 50;  // Sensor range [m]

    // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_pos [3] = {0.3, 0.3, 0.01};
    // Landmark measurement uncertainty [x [m], y [m]]
    double sigma_landmark [2] = {0.3, 0.3};

    // Read map data
    Map map;
    if (!read_map_data("../data/map_data.txt", map)) {
        std::cout << "Error: Could not open map file" << std::endl;
        return -1;
    }

    pf.updateWeights(sensor_range, sigma_landmark, observ, map);

    return 0;

}