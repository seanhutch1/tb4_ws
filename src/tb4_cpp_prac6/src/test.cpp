




void 
AmclNode::pf_resample(pf_t * pf){
  // Define pointers to two particle sets
  pf_sample_set_t * old_particle_set, * new_particle_set;
  // Define pointers to samples of two sets
  pf_sample_t * sample_in_old_set, * sample_in_new_set;
  // Initialise two particle sets
  old_particle_set = pf->sets + pf->current_set;
  new_particle_set = pf->sets + (pf->current_set + 1)%2;
  // Initialise histogram of of new particle set in terms of Kd tree
  iar_amcl::pf_kdtree_clear(new_particle_set->kdtree);
  // Define and initliase cumulative distribution of probability of old particle set
  double * cd_old_set; 
  cd_old_set = (double *)malloc(sizeof(double) * (old_particle_set->sample_count + 1));

  /* TODO TASK - MILESTONE # 4.1
    Calculate the cumulative importance weight here for resampling purpose, and save 
    to the array "cd_old_set"
  */


  /* TODO TASK - MILESTONE #1
    Initialise new sample set with sample count = 0, and  
    set total weight of the new particle set to zero
  */


  /* TODO TASK - MILESTONE #2
    calculate w_diff = 1 - w_fast /w_slow, and if w_diff < 0, set it to zero
  */


  while(new_particle_set->sample_count < pf->max_samples)
  {
    sample_in_new_set = new_particle_set->samples + new_particle_set->sample_count;

    if(drand48() < w_diff){
      /* TODO TASK - MILESTONE #3
        Generate uniformly distributed samples according to a probability of w_diff
      */
    } 
    else {
      /* TODO TASK - MILESTONE # 4.2
        Generate samples based on weight 
      */
    }
    /* TODO TASK - MILESTONE # 5
      Allocate weights to new particles, and calculate total weights
    */


    iar_amcl::pf_kdtree_insert(new_particle_set->kdtree, sample_in_new_set->pose, sample_in_new_set->weight);
    new_particle_set->sample_count++;

    /* TODO TASK - MILESTONE # 6
      Compute the value of "M", i.e., the number of samples that the KL distance between particles and real
      distribution is less than "epsilon" with probability of "1 - delta"
    */
    int M;
    int k = new_particle_set->kdtree->leaf_count;

    /* TODO TASK - MILESTONE # 7
      Check whether number of samples in the new particle set is larger than M, if yes, break the resampling.
    */
  }


  if(w_diff > 0.05)
    pf->w_fast = pf->w_slow = 0.0;

  /* TODO TASK - MILESTONE # 8
    Normalised the weights in the new particle set
  */


   pf_cluster_stats(pf, new_particle_set);
   pf->current_set = (pf->current_set + 1) % 2;
   free(cd_old_set);
}

}  // namespace iar_amcl
