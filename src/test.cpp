/**
 * test.cpp
 * used for testing methods
 */

#include <iostream>
#include <random> // Need this for sampling from distributions
#include "particle_filter.h"

using std::normal_distribution;

/**
 * Prints samples of x, y and theta from a normal distribution
 * @param gps_x   GPS provided x position
 * @param gps_y   GPS provided y position
 * @param theta   GPS provided yaw
 */
void printSamples(double gps_x, double gps_y, double theta);


int main() {

    // Landmark measurement uncertainty [x [m], y [m]]
    double std [3] = {0.3, 0.3, 0.4};
    double std_2[3] = {0.5, 0.5, 0.5};

    // Create particle filter
    ParticleFilter pf;
    pf.init(1,1,1,std);
    pf.prediction(1,std_2,1,1);

    return 0;

}