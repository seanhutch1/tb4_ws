#include <math.h>
#include <assert.h>

#include "iar_amcl/likelihood_field_model.hpp"

namespace iar_amcl
{
    /// class constructor
    LikelihoodFieldModel::LikelihoodFieldModel(
    double z_hit, double z_max, double z_rand, double sigma_hit,
    double max_occ_dist, size_t max_beams, map_t * map)
    : Laser(max_beams, map)
    {
        z_hit_ = z_hit;
        z_rand_ = z_rand;
        sigma_hit_ = sigma_hit;
        z_max_ = z_max;
        map_update_cspace(map, max_occ_dist); /// LINE 2 of algorithm 1. this is the pre-computation that computes the likelihood field map.
    }

    // Determine the probability for the given pose
    double
    LikelihoodFieldModel::sensorFunction(nav2_amcl::LaserData * data, pf_sample_set_t * set)
    {
        LikelihoodFieldModel * self;
        int i, j, step;
        double pz; 
        double dist;
        double p; // probability (or weight) of indivdual particle
        double obs_range, obs_bearing; // the measurement values of one laser beam
        double total_weight;
        pf_sample_t * sample;
        pf_vector_t pose_robot; 

        // hit_x and hit_y are the horizontal position and vertical position of the endpoint of a laser beam
        double hit_x, hit_y;

        self = reinterpret_cast<LikelihoodFieldModel *>(data->laser);

        total_weight = 0.0;

        // Compute the sample weights
        for (j = 0; j < set->sample_count; j++) {
                sample = set->samples + j;
                pose_robot = sample->pose;

                /* TODO TASK - MILSTONE 1.1 
                    Compute the pose of the lidar sensor.
                */

                pf_vector_t lidar_pose_to_robot = self->laser_pose_; /// Tsens = {xT, yT, thetaT} = sensor in robot's frame

                const double xr = pose_robot.v[0];
                const double yr = pose_robot.v[1];
                const double thetar = pose_robot.v[2];

                const double xT = lidar_pose_to_robot.v[0];
                const double yT = lidar_pose_to_robot.v[1];
                const double thetaT = lidar_pose_to_robot.v[2];

                const double xs;
                const double ys;
                const double thetas;

                xs = xr + xT * cos(thetar) - yT * sin(thetar);
                ys = yr + xT * cos(thetar) + yT * sin(thetar);
                thetas = thetar + thetaT;

            


                p = 1.0;

                step = (data->range_count - 1) / (self->max_beams_ - 1);
                // Step size normally larger than 1, but in case of error setting max number beams used 
                // to model the probability, we need to check the step and ensure it is larger or equal to 1
                if (step<1)
                    step = 1;

                for (i = 0; i < data->range_count; i += step) {
                    obs_range = data->ranges[i][0];
                    obs_bearing = data->ranges[i][1];

                    pz = 0.0;
                    /* TODO TASK - MILESTONE 1.2
                        Check whether the laser beam's value is infinity, if yes, skip this beam measurement
                    */

                    if (isinf(obs_range)) {
                        continue;
                    }

                    if (isnan(obs_range)) {
                        continue;
                    }



                    /* 
                        TODO TASK - MILESTONE 1.3
                        Check whether a failure measurement is detected, i.e., max range is detected. If yes, update the probability
                    */
                    if (obs_range == data->range_max) {
                        pz = pz + self->z_max_
                    }




                    /* 
                        TODO TASK - MILESTONE 1.4
                        Process a beam with range measurement less than the maximum value.
                    */

                    if (obs_range < data->range_max) {

                        pz = pz + ( self->z_rand_ * (1 / data->range_max ));

                        const double xhit;
                        const double yhit;

                        xhit = xs + obs_range * cos(thetas + obs_bearing);
                        yhit = ys + obs_range * sin(thetas + obs_bearing);

                        int m_x = MAP_GXWX(self->map_, xhit); /// convert from global frame to map frame
                        int m_y = MAP_GYWY(self->map_, yhit); ///

                        bool valid = MAP_VALID(self->map_, m_x, m_y); /// check if point lies within map boundary

                        if (valid) {
                            
                            int index = MAP_INDEX(self->map_, m_x, m_y); /// get the map cell's index number
                            dist = self->map_->cells[index].occ_dist; /// get the map cell's distance to nearest obstacle.
                            


                        }
                        else {
                            dist = self->map_->max_occ_dist;
                        }
                        

                        pz = pz + self->z_hit_ * exp(-(dist * dist) / (2.0 * self->sigma_hit_ * self->sigma_hit_) )




                    }


                    
                    assert(pz <= 1.0);
                    assert(pz >= 0.0);
                    //      p *= pz;
                    // here we have an ad-hoc weighting scheme for combining beam probs
                    // works well, though...
                    p += pz * pz * pz;
                }
            sample->weight *= p;
            total_weight += sample->weight;
        }
        return total_weight;
    }

    bool 
    LikelihoodFieldModel::sensorUpdate(pf_t * pf, nav2_amcl::LaserData * data)
    {
        if (max_beams_ < 2) {
            return false;
        }
        pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

        return true;
    }
}