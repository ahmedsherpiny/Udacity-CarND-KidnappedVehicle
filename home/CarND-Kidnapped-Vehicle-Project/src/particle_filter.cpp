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
#include<map>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 500;  // TODO: Set the number of particles
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  particles.clear();
  for(int i = 0; i < num_particles; ++i) {  

      Particle p;
      p.id = i + 1;
      p.weight = 1;
      p.x = dist_x(gen);
      p.y = dist_y(gen);
      p.theta = dist_theta(gen);
      
      particles.push_back(p);
  }
  //is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  double x_new, y_new, theta_new;
  default_random_engine gen;

  for (int i = 0; i < num_particles; ++i){
    Particle *p = &particles.at(i);
    x_new = p->x + (velocity * (sin(p->theta + yaw_rate * delta_t) - sin(p->theta)) / yaw_rate);
    y_new = p->y + (velocity * (cos(p->theta) - cos(p->theta + yaw_rate * delta_t)) / yaw_rate);
    theta_new = p->theta + yaw_rate * delta_t;
    
    normal_distribution<double> x_dist(x_new, std_pos[0]);
    normal_distribution<double> y_dist(y_new, std_pos[1]);
    normal_distribution<double> theta_dist(theta_new, std_pos[2]);
    p->x = x_dist(gen);
    p->y = y_dist(gen);
    p->theta = theta_dist(gen);
  }



}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  double min_dist, curr_dist;
  //int min_pred_idx;
  for(unsigned int i = 0; i < observations.size(); ++i){
    LandmarkObs *obs = &observations.at(i);
    min_dist = std::numeric_limits<int>::max();
    obs->id = -1;
    for(unsigned int j = 0; j < predicted.size(); ++j){  
      curr_dist = dist(obs->x, obs->y, predicted.at(j).x, predicted.at(j).y);
      if(curr_dist < min_dist){
        min_dist = curr_dist;
        obs->id = predicted.at(j).id;
        //min_pred_idx = j;
      }
    }
    //if (min_dist > 100) std::cout<< i << " " << obs->id << " "<< min_dist<<std::endl;
    /*if(min_dist > 100){
      for(auto m:predicted){
        std::cout<< "possible preiction: "<<m.id<<" "<< m.x<<" "<< m.y<<std::endl;
      }
    }*/
    //std::cout<<obs->id<<" "<<obs->x << " " << predicted.at(min_pred_idx).x << " " << obs->y << " " << predicted.at(min_pred_idx).y << std::endl;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: in_dist >50
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htmh
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.eduhttp://planning.cs.uiuc.edu/node99.html/node99.html
   */
  vector<LandmarkObs> predicted;
  //double x_c, y_c; //landmark position in vehicle coordinates
  double x_m, y_m; //observation x and y in map coordinates
  vector<int> associations;
  vector<double> sense_x, sense_y;
  //double weights_sum = 0.0;
  
  for(int i = 0; i < num_particles; ++i){
      
    Particle *p = &particles.at(i);
    vector<LandmarkObs> p_observations = observations;
    //transform observations to map coordinates
    for(unsigned int k = 0; k < p_observations.size(); ++k){
      LandmarkObs *obs = &p_observations.at(k);
      x_m = p->x + (cos(p->theta) * obs->x) - (sin(p->theta) * obs->y);
      y_m = p->y + (sin(p->theta) * obs->x) + (cos(p->theta) * obs->y);
      obs->x = x_m;
      obs->y = y_m;
    }
    
    predicted.clear();
    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j){
      Map::single_landmark_s lm = map_landmarks.landmark_list.at(j);
      //x_c = -p->x + (cos(-p->theta) * lm.x_f) - (sin(-p->theta) * lm.y_f);
      //y_c = -p->y + (sin(-p->theta) * lm.x_f) + (cos(-p->theta) * lm.y_f);
      //if(dist(x_c, 0, y_c, 0) < sensor_range){
      if(dist(lm.x_f, lm.y_f, p->x, p->y) <= sensor_range){
      
        LandmarkObs predicted_lm;
       
        //transfer prediction to vehicle coordinates
        //predicted_lm.x = -p->x + (cos(-p->theta) * lm.x_f) - (sin(-p->theta) * lm.y_f);
        //predicted_lm.y = -p->y + (sin(-p->theta) * lm.x_f) + (cos(-p->theta) * lm.y_f);
        predicted_lm.x = lm.x_f;
        predicted_lm.y = lm.y_f;
        //predicted_lm.x = x_c;
        //predicted_lm.y = y_c;
        predicted_lm.id = lm.id_i;
        predicted.push_back(predicted_lm);
      }
    }
    dataAssociation(predicted, p_observations);
    //prepare ssociations
    associations.clear();
    sense_x.clear();
    sense_y.clear();

    p->weight = 1;
    for(unsigned int k = 0; k < p_observations.size(); ++k){
      
      LandmarkObs obs = p_observations.at(k);
      //x_m = p->x + (cos(p->theta) * obs.x) - (sin(p->theta) * obs.y);
      //y_m = p->y + (sin(p->theta) * obs.x) + (cos(p->theta) * obs.y);
      if(obs.id > 0){
        associations.push_back(obs.id);
        x_m = obs.x;
        y_m = obs.y;
        sense_x.push_back(x_m);
        sense_y.push_back(y_m);
          
        //update weight based onthe observation
        double pred_x = map_landmarks.landmark_list.at(obs.id - 1).x_f;
        double pred_y = map_landmarks.landmark_list.at(obs.id - 1).y_f;
        double exponent = (pow(x_m - pred_x, 2) / (2 * pow(std_landmark[0], 2))) + (pow(y_m - pred_y, 2) / (2 * pow(std_landmark[1], 2)));
        //std:ut <<exponent<<std::endl;
        //std::cout<<x_m-pred_x<< " " << y_m -pred_y << std::endl;
        //std::cout<<obs.id<< " "<< map_landmarks.landmark_list.at(obs.id - 1).id_i<<" "<<x_m<<" "<<pred_x<<" "<<y_m<<" "<<pred_y<<std::endl;
        p->weight *= (exp(-exponent) / (2 * M_PI * std_landmark[0] * std_landmark[1]));
        //if(p->weight == 0) std::cout<<obs.id<< " " << p->x << " " << p->y << " " <<p->theta<< " " <<obs.x<< " "<<obs.y<<" "<< map_landmarks.landmark_list.at(obs.id - 1).id_i<<" "<<x_m<<" "<<pred_x<<" "<<y_m<<" "<<pred_y<<std::endl;
        //if(p->weight == 0) p->weight = std::numeric_limits<double>::min();
        //if(exponent>50) std::cout << predicted.size() << " " << p->x << " " << p->y<<std::endl;
      }
    }
    SetAssociations(*p, associations, sense_x, sense_y);
    // weights_sum += p->weight;
    //std::cout << p->weight;
  }
  //normalize weights
  /*for(int l = 0; l < num_particles; ++l){
    Particle *p = &particles.at(l);
    p->weight /= weights_sum;
  }*/                   

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */


  std::random_device rd;
  std::mt19937 gen(rd());
  //std::initializer_list<double> weights;
  vector<double> weights_v;
  vector<Particle> resampled_particles;
  for(auto pa:particles){
  
    double w = pa.weight;
    weights_v.push_back(w);
    //Weights.(p.weight);
  }
  std::discrete_distribution<> d(weights_v.begin(), weights_v.end());
  for(int n = 0; n < num_particles; ++n){
  
    Particle new_p = particles.at(d(gen));
    
    new_p.id = n + 1;
    /*new_p.x = p->x;
    new_p.y = p->y;
    new_p.theta = p->theta;
    new_p.weight = p->weight;*/
    resampled_particles.push_back(new_p);
  }
  particles = resampled_particles;
  
    /*std::map<int, int> m;
    for(int n=0; n< num_particles; ++n) {
        ++m[d(gen)];
    }
  for(auto t : m) {
        std::cout << t.first << " generated " << t.second << " times\n";
    }*/
  
  
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