/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *
 * Updated: Jul 1, 2019
 * Modifier: Marcus Neuert
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "map.h"
#include <cmath>

using std::string;
using std::vector;
using std::normal_distribution;

static std::default_random_engine randomEngine;
static bool debug = false;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
     * Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     * Add random Gaussian noise to each particle.
     * NOTE: Consult particle_filter.h for more information about this method
     *   (and others in this file).
     */
    num_particles = 100;  // TODO: Set the number of particles
    normal_distribution<double> std_x(x, std[0]);
    normal_distribution<double> std_y(y, std[1]);
    normal_distribution<double> std_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        // add gausian noice to each particle
        Particle p;
        p.id = i;
        p.x = std_x(randomEngine);
        p.y = std_y(randomEngine);
        p.theta = std_theta(randomEngine);
        p.weight = 1.0;

        particles.push_back(p);
        weights.push_back(p.weight);

        if (debug) {
            std::cout << "Sample " << i + 1 << " " << p.x << " " << p.y << " " << p.theta << std::endl;
        }
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    /**
     * Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */


    //calculate new state of each particle
    for (int i = 0; i < num_particles; i++) {
        if (fabs(yaw_rate) < 0.001) {
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        } else {
            particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t)
                                                     - sin(particles[i].theta));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta)
                                                     - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        normal_distribution<double> std_x(particles[i].x, std_pos[0]);
        normal_distribution<double> std_y(particles[i].y, std_pos[1]);
        normal_distribution<double> std_theta(particles[i].theta, std_pos[2]);
        // add noise
        particles[i].x = std_x(randomEngine);
        particles[i].y = std_y(randomEngine);
        particles[i].theta = std_theta(randomEngine);

        if (debug) {
            std::cout << "Sample " << i + 1 << " " << particles[i].x << " " << particles[i].y << " "
                      << particles[i].theta << std::endl;
        }
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {
    /**
     * Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */


    //iterate over all observations
    for (unsigned int i = 0; i < observations.size(); i++) {

        //init distance to maximum value for the case that no neighbor is found
        double distance = std::numeric_limits<double>::max();
        //init matching id to none existing value
        int matching_id = -1;

        //iterate over all predictions
        for (unsigned int j = 0; j < predicted.size(); j++) {

            //calc distance between observation and prediction
            double current_distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            //store min distance and id of prediction
            if (current_distance < distance) {
                distance = current_distance;
                matching_id = predicted[i].id;
            }
        }
        observations[i].id = matching_id;

        if (debug) {
            std::cout << "Matching points " << i << " " << observations[i].id << std::endl;
        }
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    /**
     *  Update the weights of each particle using a multi-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */

    double weights_sum = 0.0;
    //
    for (unsigned int i = 0; i < particles.size(); i++) {
        double Xmap, Ymap;

        //
        std::vector<int> associations;
        std::vector<double> sense_x;
        std::vector<double> sense_y;
        //
        associations.clear();
        sense_x.clear();
        sense_y.clear();
        //
        for (unsigned int j = 0; j < observations.size(); j++) {

            // transfrom sensor data to map coordinate system
            Xmap = particles[i].x + (cos(particles[i].theta) * observations[j].x) -
                   (sin(particles[i].theta) * observations[j].y);
            Ymap = particles[i].y + (sin(particles[i].theta) * observations[j].x) +
                   (cos(particles[i].theta) * observations[j].y);

            // map the observation with landmark id
            double dist_min, dist_tmp;
            int id_min = -1;

            for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {
                dist_tmp = dist(Xmap, Ymap, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
                if (k == 0) {
                    dist_min = dist_tmp;
                    id_min = k;
                }
                if (dist_min > dist_tmp) {
                    dist_min = dist_tmp;
                    id_min = k;
                }
            }
            // ignore landmark if not in range
            if (id_min == -1) {
                continue;
            }

            associations.push_back(map_landmarks.landmark_list[id_min].id_i);
            sense_x.push_back(Xmap);
            sense_y.push_back(Ymap);

            //caculate the weight
            double mu_x, mu_y, sig_x, sig_y, x_obs, y_obs;

            x_obs = Xmap;
            y_obs = Ymap;
            sig_x = std_landmark[0];
            sig_y = std_landmark[1];
            mu_x = map_landmarks.landmark_list[id_min].x_f;
            mu_y = map_landmarks.landmark_list[id_min].y_f;

            // calculate normalization term
            double gauss_norm;
            gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

            double exponent;
            exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
                       + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

            double weight;
            weight = gauss_norm * exp(-exponent);
            //update the weight
            particles[i].weight *= weight;

        }
        //associate the observation with landmark
        SetAssociations(particles[i], associations, sense_x, sense_y);
        //
        weights_sum += particles[i].weight;
    }

    // Normalize the weights for all particles.
    for (unsigned int i = 0; i < particles.size(); i++) {
        particles[i].weight = particles[i].weight / weights_sum;
    }
}

void ParticleFilter::resample() {
    /**
     * Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
    vector<Particle> new_particles;

    // get all weights
    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }

    // generate random starting index
    std::uniform_int_distribution<int> uniintdist(0, num_particles - 1);
    auto index = uniintdist(randomEngine);

    // define the maximum weight
    double max_weight = *max_element(weights.begin(), weights.end());

    // uniform random distribution [0.0, max_weight)
    std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

    double beta = 0.0;

    // resample
    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(randomEngine) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }

    particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

