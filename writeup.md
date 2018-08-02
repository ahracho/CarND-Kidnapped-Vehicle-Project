# **Kidnapped Vehicle Project**

The goals / steps of this project are the following:
* Understand how Particle Filter works in Localization
* Implement Particle Filter in C++


[//]: # (Image References)

[image1]: ./writeup_images/final_result.png "Final Result"
[image2]: ./writeup_images/particle_filter_process.png "Process"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/747/view) individually and describe how I addressed each point in my implementation.  

### Accuracy
#### 1. Does your particle filter localize the vehicle to within the desired accuracy?
I completed the project under Ubuntu bash on Windows 10. I didn't modify CMakeLists.txt and other configuration files, so follow below to compile the code.

~~~sh
cd build
cmake ..
make
./partcle_filter
~~~

Then, launch Term 2 simulator. And the final result is as below.  
![image1]

### Performance
#### Does your particle run within the specified time of 100 seconds?
As seen above, I finished the whole step in 50 seconds.


### General
#### Does your code use a particle filter to localize the robot?  
![image2]  

I followed the general particle filter process provided in the lectures.  

**1. Initialization**  
For initialization, use x, y, theta value from GPS and generate particles. I use 50 particles for this project (at first, I tried 100 particles but difference was not significant even with less particles).

~~~cpp
// particle_filter.cpp
void ParticleFilter::init(double x, double y, double theta, double std[]) {
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
~~~  
  
**2. Prediction**  
For every observation, velocity and yaw rate was given, and I used these value for prediction step. Iterating every particles and update x, y, theta values. I added noise factors to the prediction and each noise factor follows N(0, std).
- noise_x ~ N(0, std_x)
- noise_y ~ N(0, std_y)
- noise_theta ~ N(0, std_theta)

~~~cpp
if (yaw_rate != 0.0) {
    particles[i].x = x + velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta)) + noise_x;
    particles[i].y = y + velocity / yaw_rate * (-cos(theta + yaw_rate * delta_t) + cos(theta)) + noise_y;
    particles[i].theta = theta + yaw_rate * delta_t + noise_theta;
}
// go straight
else { 
    particles[i].x = x + cos(theta)*velocity*delta_t + noise_x;
    particles[i].y = y + sin(theta)*velocity *delta_t + noise_y;
    particles[i].theta = theta + noise_theta;
}
~~~

**3. Weight Update**  
**3.1. Trasformation and Association**  
After prediction, I update weight of each particle. Before calculation, observation measurements which are vehicle coordinate should be converted into map coordinate.
~~~cpp
double map_x = cos(theta)*obs_x - sin(theta)*obs_y + x;
double map_y = sin(theta)*obs_x + cos(theta)*obs_y + y;
~~~

To find nearest landmark for each observation, I use dataAssociation() function. I modified parameters of this function to include sensor range, landmarks and observations.
~~~cpp
// for each observation
for (unsigned int i = 0; i < observations.size(); i++) {
    double min = sensor_range;
    int id = -1;

    // find cloest landmark within sensor range
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
~~~

**3.2. Update Weight**  
After association, I need to update weight. Comparing observation measurements and matching landmarks, and then calculate multi-variate Gaussian distribution.   
~~~cpp
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
}
~~~  

**4. Resampling**  
After updating weight, need to resample so that particles with higher weight (higher accuracy) obtains higher proportion in particles.
~~~cpp
default_random_engine gen;
std::discrete_distribution<> weight_dist(weights.begin(), weights.end());

std::vector<Particle> resampled;
for (int i = 0; i < num_particles; i++) {
    int index = weight_dist(gen);
    resampled.push_back(particles[index]);
}
particles = resampled;
~~~