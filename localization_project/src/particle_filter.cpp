/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *  Modified on March 1, 2018
 *      Author: Rob
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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
	// set amount of particles
	num_particles = 101;

	// create randomizer for noise
	default_random_engine randomizer;

	// create normal distributions with specific std. deviations
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// iterate through amount of defined particles
	for(int i=0; i < num_particles; i++)
	{
		Particle temp_particle;
		// assign increment as id
		temp_particle.id = i;
		// init x, y, and theta with normal distribution generator
		temp_particle.x = dist_x(randomizer);
		temp_particle.y = dist_y(randomizer);
		temp_particle.theta = dist_theta(randomizer);
		// init weights with 1
		temp_particle.weight = 1;
		// store temporary particle in Object
		particles.push_back(temp_particle);
		// assign global weights to 1
		weights.push_back(temp_particle.weight);
	}
	// end of init phase
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	// iterate through amount of defined particles
	for(int i=0; i < num_particles; i++)
	{
		double updated_x;
		double updated_y;
		double updated_theta;

		// special case: yaw rate is 0
		if ( yaw_rate == 0 )
		{
			updated_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			updated_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			updated_theta = particles[i].theta;
		}
		// normal case: yaw rate is > 0
		else
		{
			updated_x = particles[i].x + (velocity / yaw_rate) *
					(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			updated_y = particles[i].y + (velocity / yaw_rate) *
					(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			updated_theta = particles[i].theta + yaw_rate * delta_t;
		}

		// create randomizer for noise
		default_random_engine randomizer;

		// create normal distributions with specific std. deviations
		normal_distribution<double> dist_x(updated_x, std_pos[0]);
		normal_distribution<double> dist_y(updated_y, std_pos[1]);
		normal_distribution<double> dist_theta(updated_theta, std_pos[2]);

		// assign particle values to object with Gaussian noise
		particles[i].x = dist_x(randomizer);
		particles[i].y = dist_y(randomizer);
		particles[i].theta = dist_theta(randomizer);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
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
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
