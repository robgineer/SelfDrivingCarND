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

	// create randomizer for noise
	default_random_engine randomizer;

	// set amount of particles
	num_particles = 100;

	// create normal distributions with specific std. deviations
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// iterate through amount of defined particles
	for(unsigned int i = 0; i < num_particles; i++)
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

	// create randomizer for noise
	default_random_engine randomizer;

	// create normal distributions with mean = 0 and specific std. deviations (used for Gaussian noise)
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	// iterate through amount of defined particles
	for(unsigned int i = 0; i < num_particles; i++)
	{
		// special case: yaw rate is 0 (represented as x < 0.0001 due to floating point inaccuracy)
		if ( fabs(yaw_rate) < 0.0001 )
		{
			particles[i].x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
		}
		// normal case: yaw rate is > 0
		else
		{
			particles[i].x 		= particles[i].x + velocity / yaw_rate *
									(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y 		= particles[i].y + velocity / yaw_rate *
									(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta 	= particles[i].theta + yaw_rate * delta_t;
		}
		// add Gaussian noise
		particles[i].x 		= particles[i].x 	+ dist_x(randomizer);
		particles[i].y 		= particles[i].y 	+ dist_y(randomizer);
		particles[i].theta 	= particles[i].theta + dist_theta(randomizer);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// define variable for minimum distance. Updated for each observation.
	double minimum_distance;
	// iterate through all observations
	for(unsigned int i = 0; i < observations.size(); i++)
	{
		// set a high initial minimum distance
		minimum_distance = 999;

		// iterate through all predictions
		for(unsigned int j = 0; j < predicted.size(); j++)
		{
			// get Eucl. distance between observed and predicted point
			double calc_distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			// check if distance is small enough for association
			if(calc_distance < minimum_distance)
			{
				//associate
				observations[i].id = predicted[j].id;
				// update min distance for this observation
				minimum_distance = calc_distance;
			}
		}
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

	// iterate through amount of defined particles
	for(int i = 0; i < num_particles; i++)
	{
		// store attributes for easy access
		double particle_x 		= particles[i].x;
		double particle_y 		= particles[i].y;
		double particle_theta 	= particles[i].theta;

		// create a list of landmarks that are visible by the vehicles sensors
		vector<LandmarkObs> filtered_map_landmarks;

		// iterate through all landmarks in the map
		for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			// store attributes for easy access
			int landmark_id 		= map_landmarks.landmark_list[j].id_i;
			float landmark_x 	= map_landmarks.landmark_list[j].x_f;
			float landmark_y 	= map_landmarks.landmark_list[j].y_f;

			// TODO: convert float to double prior to distance calculation
			//get Eucledian distance from particle to landmark in map
			double distance = dist(particle_x, particle_y, landmark_x, landmark_y);

			// check if distance is in sensor range
			if (distance <= sensor_range)
			{
				// store potentially visible landmarks into list
				LandmarkObs selected_landmark = {landmark_id, landmark_x, landmark_y};
				filtered_map_landmarks.push_back(selected_landmark);
			}
		}
		// create a vector of observations in map coordinates
		vector<LandmarkObs> observations_in_map_coordinates;

		// transform observations into map coordinates
		for(unsigned int k = 0; k < observations.size(); k++)
		{
			// homogeneous transformation of observations
			double x_m = particle_x + cos(particle_theta) * observations[k].x - sin(particle_theta) * observations[k].y;
			double y_m = particle_y + sin(particle_theta) * observations[k].x + cos(particle_theta) * observations[k].y;
			// transformed observation
			LandmarkObs transformed_ob = {observations[k].id, x_m, y_m};
			observations_in_map_coordinates.push_back(transformed_ob);
		}

		// update observations with IDs of nearest (in range) MAP landmarks
		dataAssociation(filtered_map_landmarks, observations_in_map_coordinates);

		// re-init weight of current particle (probability that the vehicle is at the position of the current particle is 100%)
		particles[i].weight = 1.0;

		// extract sigma values for x and y
		double sigma_x = std_landmark[0];
		double sigma_y = std_landmark[1];

		// search for match in observation and in range landmarks
		for(unsigned int l = 0; l < observations_in_map_coordinates.size(); l++)
		{
			for(unsigned int m = 0; m < filtered_map_landmarks.size(); m++)
			{
				if(observations_in_map_coordinates[l].id == filtered_map_landmarks[m].id)
				{// match found
					// extract observation position
					double obs_x = observations_in_map_coordinates[l].x;
					double obs_y = observations_in_map_coordinates[l].y;

					// extract in range landmark positions
					double landmark_x = filtered_map_landmarks[m].x;
					double landmark_y = filtered_map_landmarks[m].y;

					// calculate multivariate Gaussian probability
					double weight = (1/(2 * M_PI * sigma_x * sigma_y)) *
							exp(- (	(pow(obs_x - landmark_x, 2)) / (2 * pow(sigma_x, 2)) +
								    (pow(obs_y - landmark_y, 2)) / (2 * pow(sigma_y, 2)) ) );

					// update probability of vehicle being at the particles position
					particles[i].weight = particles[i].weight * weight;
				}
			}
			// store weights for easy access in resampling
			weights[i]=particles[i].weight;
		}
	}
}

void ParticleFilter::resample() {
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	/*** This elegant solution has been taken over from the Q+A video.  ***/

	// create randomizer for noise
	default_random_engine randomizer;

	// create a distribution with weights as normalization values
	discrete_distribution<int> dist_discrete(weights.begin(), weights.end());

	vector<Particle> resampled_particles;

	for(unsigned int i = 0; i < num_particles; i++)
	{
		resampled_particles.push_back(particles[dist_discrete(randomizer)]);
	}
	particles = resampled_particles;
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
