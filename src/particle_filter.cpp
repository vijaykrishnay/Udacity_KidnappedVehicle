/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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

using std::string;
using std::vector;
using std::normal_distribution;
using std::sin;
using std::cos;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles.
   * Initialize all particles to first position (based on estimates of x, y,
   * theta and their uncertainties from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 1000;  // Set the number of particles
  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> heading_theta(theta, std[2]);
  for (int i = 0; i < num_particles; i++){
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = heading_theta(gen);
    p.weight = 1;
    particles.push_back(p);
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
  std::default_random_engine gen;

  normal_distribution<double> noise_x(0., std_pos[0]);
  normal_distribution<double> noise_y(0., std_pos[1]);
  normal_distribution<double> noise_theta(0., std_pos[2]);
  double x_new, y_new, theta_new;
  Particle p;
  if (yaw_rate == 0.){
    yaw_rate = 1e-4; // take care of singulatiry
  }
  for (int i = 0; i < num_particles; i++){
    p = particles[i];

    // Compute new position of particles
    x_new = p.x + velocity/yaw_rate*(sin(p.theta + yaw_rate*delta_t) - sin(p.theta));
    y_new = p.y + velocity/yaw_rate*(cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
    theta_new = p.theta + yaw_rate*delta_t;

    // Add noise and set new particle positions
    particles[i].x = x_new + noise_x(gen);
    particles[i].y = y_new + noise_y(gen);
    particles[i].theta = theta_new + noise_theta(gen);
  }
  // std::cout << "predicted" << std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  double min_dist, dist_tmp;
  for (int i = 0; i < observations.size(); i++){
    for (int j = 0; j < predicted.size(); j++){
      dist_tmp = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (j==0){
        min_dist = dist_tmp;
        observations[i].id = j;
      }
      if (dist_tmp < min_dist){
        min_dist = dist_tmp;
        // std::cout << i << ", " << j << ", " << dist_tmp << ", " << predicted[j].id << std::endl;
        observations[i].id = j;
      }
      if (min_dist == 0){
        break; // best match found!
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
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
  vector<LandmarkObs> predicted_landmarks;
  vector<LandmarkObs> map_observations;
  vector<double> sense_x;
  vector<double> sense_y;
  vector<int> associations;

  Particle p;
  LandmarkObs l;
  double *xy_vals;
  double weight;
  int assoc;
  double mu_x, mu_y;
  
  // Extract landmarks to predicted landmarks (in Map Coords)
  for (int j = 0; j < map_landmarks.landmark_list.size(); j++){
    l.id = map_landmarks.landmark_list[j].id_i;
    l.x = map_landmarks.landmark_list[j].x_f;
    l.y = map_landmarks.landmark_list[j].y_f;
    predicted_landmarks.push_back(l);
    // if (dist(p.x, p.y, l.x, l.y) < 2*sensor_range){
    //   predicted_landmarks.push_back(l);
    // }
  }

  for (int i = 0; i < num_particles; i++){
    p = particles[i];
    map_observations.clear();
    
    // std::cout << "uw1" << std::endl;
    // Transform observations to map coords based on particle position
    for (int j = 0; j < observations.size(); j++){
      xy_vals = transformToMap(p.x, p.y, p.theta, observations[j].x, observations[j].y);
      l.id = observations[j].id;
      l.x = xy_vals[0];
      l.y = xy_vals[1];
      map_observations.push_back(l);
      if (xy_vals[0] != xy_vals[0]){
        std::cout << "obs to transfrm: " << observations[j].x << ", " << observations[j].y << std::endl;
        std::cout << "p: " << p.x << ", " << p.y << ", " << p.theta << std::endl;
      }
      // std::cout << "p vals: " << p.x << ", " << p.y << ", " << p.theta << std::endl;
      // std::cout << "o vals: " << observations[j].x << ", " << observations[j].y << std::endl;
      // std::cout << "l vals: " << l.x << ", " << l.y << std::endl;
    }

    // associate ladmarks to observations
    // std::cout << "map obs:" << map_observations.size() << std::endl;
    // std::cout << "map lmarks:" << predicted_landmarks.size() << std::endl;
    dataAssociation(predicted_landmarks, map_observations);
    weight = 1.0;
    associations.clear();
    sense_x.clear();
    sense_y.clear();
    for (int j = 0; j < map_observations.size(); j++){
      assoc = map_observations[j].id;
      // std::cout << "post assoc" << j << ", " << assoc << std::endl;

      mu_x = predicted_landmarks[assoc].x;
      mu_y = predicted_landmarks[assoc].y;
      weight *= calculateWeight(map_observations[j].x, map_observations[j].y, mu_x, mu_y, 
                                std_landmark[0], std_landmark[1]);
      associations.push_back(predicted_landmarks[assoc].id);
      sense_x.push_back(map_observations[j].x);
      sense_y.push_back(map_observations[j].y);
      // std::cout << "weight: " << weight << std::endl;
    }
    particles[i].weight = weight;
    // particles[i].associations = associations;
    SetAssociations(particles[i], associations, sense_x, sense_y);
    // std::cout << "weights: " << weight << std::endl;
    // std::cout << "map size: " << map_observations.size() << std::endl;
  }
  // std::cout << "last wt: " << weight << std::endl;
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<double> weights;
  double weight_sum = 0.;
  for (int i = 0; i < particles.size(); i++){
    weight_sum += particles[i].weight;
  }
  // std::cout << "weights sum" << weight_sum << std::endl;

  for (int i = 0; i < particles.size(); i++){
    weights.push_back(particles[i].weight/weight_sum);
  }

  std::default_random_engine generator;
  std::discrete_distribution<int> distribution(weights.begin(), weights.end());
  int random_sel;
  std::vector<Particle> particles_new(particles);

  // randomly select particles based on weight
  for (int i = 0; i < particles_new.size(); i++){
    random_sel = distribution(generator);
    particles[i] = particles_new[random_sel];
  }
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
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
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}