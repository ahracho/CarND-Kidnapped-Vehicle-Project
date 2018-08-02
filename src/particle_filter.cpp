/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <map>

#include "particle_filter.h"
#include "map.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 50;

    default_random_engine gen;
    normal_distribution<double> x_dist{x, std[0]};
    normal_distribution<double> y_dist{y, std[1]};
    normal_distribution<double> theta_dist{theta, std[2]};
    
    for (int i = 0; i < num_particles; i++) {
        Particle particle;
        particle.id = i;
        particle.x = x_dist(gen);
        particle.y = y_dist(gen);
        particle.theta = theta_dist(gen);
        particle.weight = 1;

        particles.push_back(particle);
        weights.push_back(1);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  default_random_engine gen;
  normal_distribution<double> x_dist{ 0, std_pos[0] };
  normal_distribution<double> y_dist{ 0, std_pos[1] };
  normal_distribution<double> theta_dist{ 0, std_pos[2] };

  for (int i = 0; i < num_particles; i++) {
    double noise_x = x_dist(gen);
    double noise_y = y_dist(gen);
    double noise_theta = theta_dist(gen);

    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    if (yaw_rate != 0.0) {
      particles[i].x = x + velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta)) + noise_x;
      particles[i].y = y + velocity / yaw_rate * (-cos(theta + yaw_rate * delta_t) + cos(theta)) + noise_y;
      particles[i].theta = theta + yaw_rate * delta_t + noise_theta;
    }
    else { // go straight
      particles[i].x = x + cos(theta)*velocity*delta_t + noise_x;
      particles[i].y = y + sin(theta)*velocity *delta_t + noise_y;
      particles[i].theta = theta + noise_theta;
    }
  }
}


void ParticleFilter::dataAssociation(double sensor_range, std::vector<Map::single_landmark_s> landmarks, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  for (unsigned int i = 0; i < observations.size(); i++) {
    double min = sensor_range;
    int id = -1;
    for (unsigned int j = 0; j < landmarks.size(); j++) {
      double x1 = landmarks[j].x_f;
      double y1 = landmarks[j].y_f;
      double x2 = observations[i].x;
      double y2 = observations[i].y;
      double distance = dist(x1, y1, x2, y2);
      if (distance < min) {
        min = distance;
        id = landmarks[j].id_i;
      }
    }
    observations[i].id = id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  const std::vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
  double weight_sum = 0.0;

  for (int i = 0; i < num_particles; i++) {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    std::vector<LandmarkObs> map_obs_v;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double obs_x = observations[j].x;
      double obs_y = observations[j].y;

      double map_x = cos(theta)*obs_x - sin(theta)*obs_y + x;
      double map_y = sin(theta)*obs_x + cos(theta)*obs_y + y;
      
      LandmarkObs map_obs;
      map_obs.id = -1;
      map_obs.x = map_x;
      map_obs.y = map_y;
      map_obs_v.push_back(map_obs);
    }

    dataAssociation(sensor_range, landmarks, map_obs_v);
    
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    
    double w = 1.0;
    for (unsigned int i = 0; i < map_obs_v.size(); i++) {
      int id = map_obs_v[i].id; // matching landmark ID
      double obs_x = map_obs_v[i].x; // observations in map coord
      double obs_y = map_obs_v[i].y;

      for (unsigned int l = 0; l < landmarks.size(); l++) {
        if (id == landmarks[l].id_i) {
          double mu_x = landmarks[l].x_f; // landmark position
          double mu_y = landmarks[l].y_f;
          w *= multivariateGaussian(obs_x, obs_y, mu_x, mu_y, std_landmark[0], std_landmark[1]);
          break;
        }
      }
      associations.push_back(id);
      sense_x.push_back(obs_x);
      sense_y.push_back(obs_y);
    }
    
    SetAssociations(particles[i], associations, sense_x, sense_y);
    
    particles[i].weight = w;
    weight_sum += w;
  }

  std::vector<double> normalized_weight;
  for (int i = 0; i < num_particles; i++) {
    particles[i].weight /= weight_sum;
    normalized_weight.push_back(particles[i].weight);
  }
  
  weights = normalized_weight;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;
  std::discrete_distribution<> weight_dist(weights.begin(), weights.end());

  std::vector<Particle> resampled;
  for (int i = 0; i < num_particles; i++) {
    int index = weight_dist(gen);
    resampled.push_back(particles[index]);
  }
  particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    //cout << "getAssociations : " << s << endl;
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    //cout << "getSenseX : " << s << endl;
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    //cout << "getSenseY : " << s << endl;
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
