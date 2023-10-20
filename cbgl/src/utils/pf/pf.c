/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "utils/pf/pf.h"
#include "utils/pf/pf_pdf.h"
#include "utils/pf/pf_kdtree.h"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);



// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data)
{
  int i, j;
  pf_t *pf;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  srand48(time(NULL));

  pf = calloc(1, sizeof(pf_t));

  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;

  pf->current_set = 0;
  for (j = 0; j < 2; j++)
  {
    set = pf->sets + j;

    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));

    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  //set converged to 0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t *pf)
{
  int i;

  for (i = 0; i < 2; i++)
  {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);

  return;
}

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  pf_pdf_gaussian_t *pdf;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  pdf = pf_pdf_gaussian_alloc(mean, cov);

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  pf_pdf_gaussian_free(pdf);

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  //set converged to 0
  pf_init_converged(pf);

  return;
}


// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn) (init_data);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  //set converged to 0
  pf_init_converged(pf);

  return;
}

void pf_init_converged(pf_t *pf){
  pf_sample_set_t *set;
  set = pf->sets + pf->current_set;
  set->converged = 0;
  pf->converged = 0;
}

// Introduce `num_to_insert` times the `pipeline_particle` into the population
// and delete the `num_to_delete` light-most particles from it
void pf_delete_light_particles_and_insert_new_pipeline_particles(pf_t *pf,
  pf_vector_t pipeline_particle, const double weight,
  const int num_to_delete, const int num_to_insert)
{
  // The old set (set_a) and the new one (set_b)
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b, *sample;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  double total_set_weight = 0;
  set_b->sample_count = 0;

  // sort according to weight; select N_current - num_to_delete for insertion
  // into the new population
  if (num_to_delete > 0)
  {
    pf_sample_set_sort_str_t widx_str[set_a->sample_count];
    for (int i = 0; i < set_a->sample_count; i++)
    {
      sample = set_a->samples + i;
      widx_str[i].weight = sample->weight;
      widx_str[i].index = i;
    }

    qsort(widx_str, set_a->sample_count, sizeof(widx_str[0]),
      pf_compare_samples_by_weight_desc);

    // select the first `set_a->sample_count - num_to_delete` particles:
    // the last `num_to_delete` particles are the ones with the lowest weights.
    // the former particles will be inserted into the population.
    int past_succ_idx[set_a->sample_count - num_to_delete];
    for (int i = 0; i < set_a->sample_count - num_to_delete; i++)
      past_succ_idx[i] = widx_str[i].index;


    // insert the high-weighted particles into the population
    for (int i = 0; i < set_a->sample_count - num_to_delete; i++)
    {
      sample_b = set_b->samples + i;

      // Copy particles from the original set
      sample_a = set_a->samples + past_succ_idx[i];
      sample_b->pose = sample_a->pose;
      sample_b->weight = sample_a->weight;

      total_set_weight += sample_b->weight;
      pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
      set_b->sample_count++;
    }
  }
  else // Insert all particles from set_a to set_b
  {
    for (int i = 0; i < set_a->sample_count; i++)
    {
      sample_b = set_b->samples + i;

      // Copy all particles from the original set
      sample_a = set_a->samples + i;
      sample_b->pose = sample_a->pose;
      sample_b->weight = sample_a->weight;

      total_set_weight += sample_b->weight;
      pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
      set_b->sample_count++;
    }
  }

  // insert `num_to_insert` times the pipeline particle into the population
  for (int i = 0; i < num_to_insert; i++)
  {
    sample_b = set_b->samples + set_b->sample_count;
    sample_b->pose = pipeline_particle;
    sample_b->weight = weight;

    total_set_weight += sample_b->weight;
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
    set_b->sample_count++;
  }


  // Normalize weights
  for (int i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total_set_weight;
  }

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);
}

/*******************************************************************************
 * Initialise the filter using the past set plus some new particles: the result
 * of the pipeline
 * Added by li9i, 13/05/2019
**/
void pf_introduce_new_pipeline_particles(pf_t *pf,
  pf_vector_t pipeline_particle, const double weight, const double percent)
{
  assert(percent >= 0.0);
  assert(percent < 1.0);

  pf_sample_set_t *set_active;
  set_active = pf->sets + pf->current_set;

  // How many particles to delete from the current population
  int num_to_delete = 0;

  // How many particles to insert into (population - deleted)
  int num_to_insert = 0;

  if (percent == 0.0)
  {
    num_to_insert = 1;
    if (set_active->sample_count == pf->max_samples)
      num_to_delete = 1;
    else
      num_to_delete = 0;
  }
  else
  {
    if (set_active->sample_count == pf->max_samples)
    {
      num_to_delete = percent * set_active->sample_count;
      num_to_insert = num_to_delete;
    }
    else if (set_active->sample_count <= (1-percent) * pf->max_samples)
    {
      num_to_delete = 0;
      num_to_insert = floor(percent * set_active->sample_count / (1 - percent));
    }
    else
    {
      num_to_delete = set_active->sample_count - (1-percent) * pf->max_samples;
      num_to_insert = percent * pf->max_samples;
    }
  }

  pf_delete_light_particles_and_insert_new_pipeline_particles(pf,
    pipeline_particle, weight, num_to_delete, num_to_insert);
}

int pf_update_converged(pf_t *pf)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;
    if(fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
       fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold){
      set->converged = 0;
      pf->converged = 0;
      return 0;
    }
  }
  set->converged = 1;
  pf->converged = 1;
  return 1;
}

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data)
{
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;

  (*action_fn) (action_data, set);

  return;
}


#include <float.h>
// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;

  // Compute the sample weights
  total = (*sensor_fn) (sensor_data, set);

  if (total > 0.0)
  {
    // Normalize weights
    double w_avg=0.0;
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= total;
    }
    // Update running averages of likelihood of samples (Prob Rob p258)
    w_avg /= set->sample_count;
    if(pf->w_slow == 0.0)
      pf->w_slow = w_avg;
    else
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
    if(pf->w_fast == 0.0)
      pf->w_fast = w_avg;
    else
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    //printf("w_avg: %e slow: %e fast: %e\n",
           //w_avg, pf->w_slow, pf->w_fast);
  }
  else
  {
    // Handle zero total
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  return;
}


// Update cluster stats when no resampling is performed
void pf_update_no_resample(pf_t *pf)
{
  // The set of active particles
  pf_sample_set_t* set = pf->sets + pf->current_set;

  // Clear the tree; we will insert particles anew
  pf_kdtree_clear(set->kdtree);

  pf_sample_t *sample;
  for (int i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  // Makes it possible to obtain the cluster to which each particle belongs
  pf_cluster_stats(pf, set);
}


// Resample the distribution
void pf_update_resample(pf_t *pf)
{
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  //double r,c,U;
  //int m;
  //double count_inv;
  double* c;

  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
    c[i+1] = c[i]+set_a->samples[i].weight;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  if(w_diff < 0.0)
    w_diff = 0.0;
  //printf("w_diff: %9.6f\n", w_diff);

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;

    if(drand48() < w_diff)
      sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
    else
    {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      double r;
      r = drand48();
      for(i=0;i<set_a->sample_count;i++)
      {
        if((c[i] <= r) && (r < c[i+1]))
          break;
      }
      assert(i<set_a->sample_count);

      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }

  // Reset averages, to avoid spiraling off into complete randomness.
  if(w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;

  //fprintf(stderr, "\n\n");

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);

  free(c);
  return;
}


// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1)
    return pf->max_samples;

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples)
    return pf->min_samples;
  if (n > pf->max_samples)
    return pf->max_samples;

  return n;
}


// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set)
{
  int i, j, k, cidx;
  pf_sample_t *sample;
  pf_cluster_t *cluster;

  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);

  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;

  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    //printf("%d %f %f %f\n",
      //i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;

    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = set->clusters + i;

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    //printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
           //cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
    //pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}


// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var)
{
  int i;
  double mn, mx, my, mrr;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;

  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}


// Returns a cluster of particles based on their cluster label
void pf_get_cluster(pf_t *pf, int clabel, pf_sample_set_t *c_cluster)
{
  // A pointer to the set holding all particle filter samples
  pf_sample_set_t *set;

  // A pointer to a single sample (a `clabel` cluster)
  pf_sample_t *cluster_sample;

  pf_sample_t *sample;
  int cidx;

  // The current set
  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
  {
    printf("clabel >= set->cluster_count\n");
    return;
  }

  // Identify how many particles belong to the `clabel` cluster
  // along with their position in the set
  int num_clabel_samples = 0;
  int clabel_samples_indices[set->sample_count];
  for (int i = 0; i < set->sample_count; i++)
  {
    // The i-th particle in the overall population
    sample = set->samples + i;

    // The cluster id for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);

    // We are only interested in particles within the `clabel` cluster
    if (cidx == clabel)
    {
      clabel_samples_indices[num_clabel_samples] = i;
      num_clabel_samples++;
    }
  }

  // The set to hold all particles belonging to the `clabel` cluster
  c_cluster->samples = calloc(num_clabel_samples, sizeof(pf_sample_t));

  // Select all samples belonging to the `clabel` cluster
  // and place them in `c_cluster`
  for (int i = 0; i < num_clabel_samples; i++)
  {
    // The clabel_samples_indices[i]-th particle in the overall population
    sample = set->samples + clabel_samples_indices[i];

    // Point to the next sample within `c_cluster` ...
    cluster_sample = c_cluster->samples + i;

    // and copy the pose and weight of this sample
    cluster_sample->pose = sample->pose;
    cluster_sample->weight = sample->weight;
  }

  // Set the sample count to its identified value
  c_cluster->sample_count = num_clabel_samples;
}


// Get the statistics for a particular cluster.
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov)
{
  pf_sample_set_t *set;
  pf_cluster_t *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
    return 0;

  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}


// Get statistics on a subset of a particular cluster.
// These are calculated based on the upper `percentage` x 100 of the
// population of `pf`.
// ADDED BY li9i, 19/03.2019
int pf_get_cluster_subset_stats(pf_t *pf, int clabel, double *weight,
  pf_vector_t *mean, pf_matrix_t *cov, double percentage,
  pf_sample_set_t *ret_subset)
{
  assert(clabel >= 0);

  // A pointer to the set holding all particle filter samples
  pf_sample_set_t *set;

  // A pointer to a single sample (a `clabel` cluster)
  pf_sample_t *cluster_sample;

  pf_sample_t *sample;
  int cidx;

  // The current set
  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
  {
    printf("clabel >= set->cluster_count\n");
    return 0;
  }

  // Get the cluster which corresponds to label `clabel`
  pf_sample_set_t cluster_subset;
  pf_get_cluster(pf, clabel, &cluster_subset);

  if (cluster_subset.sample_count > 0)
  {
    // At this point `cluster_subset` holds all particles from `pf` that belong
    // to the `clabel` cluster.
    // Now sort particles in this cluster based on their weight
    pf_sample_set_sort_str_t widx_str[cluster_subset.sample_count];
    for (int i = 0; i < cluster_subset.sample_count; i++)
    {
      sample = cluster_subset.samples + i;
      widx_str[i].weight = sample->weight;
      widx_str[i].index = i;
    }

    // Use qsort (stdlib)
    qsort(widx_str, cluster_subset.sample_count, sizeof(widx_str[0]),
      pf_compare_samples_by_weight_desc);

    // Now select the upper `percentage` x 100 percent of particles
    assert(percentage >= 0.0);
    assert(percentage <= 1.0);

    // How many particles to select from this cluster?
    pf_cluster_t *cluster;
    cluster = set->clusters + clabel;
    int size_subset = ceil(cluster->count * percentage);

    /*printf("size_subset = %d\n", size_subset);*/

    double total_weight = 0.0;
    double m[4];
    pf_matrix_t cov;

    // Init m, cov
    for (int i = 0; i < 4; i++)
      m[i] = 0.0;

    for (int j = 0; j < 2; j++)
      for (int k = 0; k < 2; k++)
        cov.m[j][k] = 0.0;

    // Collect weighted pose mean and covariance.
    // If percentage = 0, then select particles whose weight is larger than the
    // mean weight of the cluster.
    if (size_subset > 0)
    {
      // This is the set of particles that will be returned for visualisation
      // purposes.
      ret_subset->samples = calloc(size_subset, sizeof(pf_sample_t));
      pf_sample_t *ret_subset_sample;

      for (int i = 0; i < size_subset; i++)
      {
        // Point to the sample with index `widx_str[i].index` within the
        // `cluster_subset`
        sample = cluster_subset.samples + widx_str[i].index;

        // Update total weight (used for normalisation)
        total_weight += sample->weight;

        /*printf("weight = %f\n", sample->weight);*/

        // Accumulate weighted x,y,theta
        m[0] += sample->weight * sample->pose.v[0];
        m[1] += sample->weight * sample->pose.v[1];
        m[2] += sample->weight * cos(sample->pose.v[2]);
        m[3] += sample->weight * sin(sample->pose.v[2]);

        // Compute covariance in linear components
        for (int j = 0; j < 2; j++)
        {
          for (int k = 0; k < 2; k++)
          {
            cov.m[j][k] +=
              sample->weight * sample->pose.v[j] * sample->pose.v[k];
          }
        }

        ret_subset_sample = ret_subset->samples + i;
        ret_subset_sample->pose = sample->pose;
        ret_subset_sample->weight = sample->weight;
      }

      ret_subset->sample_count = size_subset;
    }
    else // Select particles whose weight is larger than the mean
    {
      // This is the set of particles that will be returned for visualisation
      // purposes.
      ret_subset->samples = calloc(set->sample_count, sizeof(pf_sample_t));
      pf_sample_t *ret_subset_sample;

      for (int i = 0; i < cluster->count; i++)
      {
        sample = cluster_subset.samples + widx_str[i].index;

        total_weight += sample->weight;
      }

      // The cluster's mean weight
      double mean_cluster_weight = total_weight / cluster->count;

      double total_weight_new = 0.0;
      int num_particles_above_mean = 0;
      for (int i = 0; i < cluster->count; i++)
      {
        sample = cluster_subset.samples + widx_str[i].index;

        // -0.00001 because it turns out 0.002 > 0.002 (with N = 500).
        // imagine that.
        if (sample->weight >= mean_cluster_weight - 0.00001)
        {
          num_particles_above_mean++;
          total_weight_new += sample->weight;

          // Accumulate weighted x,y,theta
          m[0] += sample->weight * sample->pose.v[0];
          m[1] += sample->weight * sample->pose.v[1];
          m[2] += sample->weight * cos(sample->pose.v[2]);
          m[3] += sample->weight * sin(sample->pose.v[2]);

          // Compute covariance in linear components
          for (int j = 0; j < 2; j++)
          {
            for (int k = 0; k < 2; k++)
            {
              cov.m[j][k] +=
                sample->weight * sample->pose.v[j] * sample->pose.v[k];
            }
          }

          ret_subset_sample = ret_subset->samples + i;
          ret_subset_sample->pose = sample->pose;
          ret_subset_sample->weight = sample->weight;
        }
      }

      ret_subset->sample_count = num_particles_above_mean ;
      total_weight = total_weight_new;
    }

    assert(total_weight > 0.0);

    // Normalise
    mean->v[0] = m[0] / total_weight;
    mean->v[1] = m[1] / total_weight;
    mean->v[2] = atan2(m[3], m[2]);

    // Covariance in linear components
    for (int j = 0; j < 2; j++)
      for (int k = 0; k < 2; k++)
        cov.m[j][k] = cov.m[j][k] / total_weight - mean->v[j] * mean->v[k];

    // Covariance in angular components
    cov.m[2][2] = -2 * log(sqrt(m[2]*m[2] + m[3]*m[3]));

    return 1;
  }
  else
    return 0;
}


// Used to compare two `pf_sample_set_sort_str_t` structures by weight in
// descending order
int pf_compare_samples_by_weight_desc(const void *str_a, const void *str_b)
{
  pf_sample_set_sort_str_t *a = (pf_sample_set_sort_str_t *)str_a;
  pf_sample_set_sort_str_t *b = (pf_sample_set_sort_str_t *)str_b;

  // Sort by DESCENDING order
  if ((*a).weight > (*b).weight)
    return -1;
  else if ((*a).weight < (*b).weight)
    return 1;
  else
    return 0;
}

// Used to compare two `pf_sample_set_sort_str_t` structures by weight in
// ascending order
int pf_compare_samples_by_weight_asc(const void *str_a, const void *str_b)
{
  pf_sample_set_sort_str_t *a = (pf_sample_set_sort_str_t *)str_a;
  pf_sample_set_sort_str_t *b = (pf_sample_set_sort_str_t *)str_b;

  // Sort by ASCENDING order
  if ((*a).weight < (*b).weight)
    return -1;
  else if ((*a).weight > (*b).weight)
    return 1;
  else
    return 0;
}
