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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <sys/types.h> // required by Darwin
#include <math.h>

#include "amcl/sensors/amcl_odom.h"

using namespace amcl;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLOdom::AMCLOdom() : AMCLSensor()
{
  this->time = 0.0;
}

void
AMCLOdom::SetModelDiff(double alpha1,
                       double alpha2,
                       double alpha3,
                       double alpha4)
{
  this->model_type = ODOM_MODEL_DIFF;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
}

void
AMCLOdom::SetModelOmni(double alpha1,
                       double alpha2,
                       double alpha3,
                       double alpha4,
                       double alpha5)
{
  this->model_type = ODOM_MODEL_OMNI;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

void
AMCLOdom::SetModel( odom_model_t type,
                    double alpha1,
                    double alpha2,
                    double alpha3,
                    double alpha4,
                    double alpha5 )
{
  this->model_type = type;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLOdom::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
  AMCLOdomData *ndata;
  ndata = (AMCLOdomData*) data;

  // 计算新的采样位姿
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);

  switch( this->model_type )
  {
  case ODOM_MODEL_OMNI:
  {
    double delta_trans, delta_rot, delta_bearing;
    double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot = ndata->delta.v[2];

    // Precompute a couple of things
    double trans_hat_stddev = (alpha3 * (delta_trans*delta_trans) +
                               alpha1 * (delta_rot*delta_rot));
    double rot_hat_stddev = (alpha4 * (delta_rot*delta_rot) +
                             alpha2 * (delta_trans*delta_trans));
    double strafe_hat_stddev = (alpha1 * (delta_rot*delta_rot) +
                                alpha5 * (delta_trans*delta_trans));

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);

      // Sample pose differences
      delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
      delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
      delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
      // Apply sampled update to particle pose
      sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                            delta_strafe_hat * sn_bearing);
      sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                            delta_strafe_hat * cs_bearing);
      sample->pose.v[2] += delta_rot_hat ;
    }
  }
  break;
  case ODOM_MODEL_DIFF:
  {
    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // 计算里程计得到的ut，即用旋转、直线运动和另一个旋转来表示相对运动
    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    // 如果原地旋转的情况，定义第一个旋转为0，认为所有旋转都是第二个旋转做的
    if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] +
            ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
      delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

    // 这里作者比书本多考虑了一种情况，就是机器人旋转小角度，然后往后走的情况，比如旋转-20°，然后往后走，计算出来可能是160°，
    // 但实际只转了20°。按书本可能是比较大的方差（160°），不太准确，但这里的话还是按照比较小的方差来采样。
    // 但我表示怀疑，万一真的转了160°再前向运动的话怎么办？还是用比较小的方差来采样，可能会使得采样的准确率下降。
    // 实际中，编码器采样率很高，且设定了一个很小的变化角度就进行运动模型更新，所以我说的这种情况几乎不会发生
    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));
    // 对每个粒子用书本的方法进行更新
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      // Sample pose differences
      delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                  this->alpha2*delta_trans*delta_trans));
      delta_trans_hat = delta_trans -
              pf_ran_gaussian(this->alpha3*delta_trans*delta_trans +
                              this->alpha4*delta_rot1_noise*delta_rot1_noise +
                              this->alpha4*delta_rot2_noise*delta_rot2_noise);
      delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                  this->alpha2*delta_trans*delta_trans));

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat *
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat *
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
    }
  }
  break;
  case ODOM_MODEL_OMNI_CORRECTED:
  {
    double delta_trans, delta_rot, delta_bearing;
    double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot = ndata->delta.v[2];

    // Precompute a couple of things
    double trans_hat_stddev = sqrt( alpha3 * (delta_trans*delta_trans) +
                                    alpha4 * (delta_rot*delta_rot) );
    double rot_hat_stddev = sqrt( alpha1 * (delta_rot*delta_rot) +
                                  alpha2 * (delta_trans*delta_trans) );
    double strafe_hat_stddev = sqrt( alpha4 * (delta_rot*delta_rot) +
                                     alpha5 * (delta_trans*delta_trans) );

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);

      // Sample pose differences
      delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
      delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
      delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
      // Apply sampled update to particle pose
      sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                            delta_strafe_hat * sn_bearing);
      sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                            delta_strafe_hat * cs_bearing);
      sample->pose.v[2] += delta_rot_hat ;
    }
  }
  break;
  case ODOM_MODEL_DIFF_CORRECTED:
  {
    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] +
            ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
      delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      // Sample pose differences
      delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(sqrt(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                       this->alpha2*delta_trans*delta_trans)));
      delta_trans_hat = delta_trans -
              pf_ran_gaussian(sqrt(this->alpha3*delta_trans*delta_trans +
                                   this->alpha4*delta_rot1_noise*delta_rot1_noise +
                                   this->alpha4*delta_rot2_noise*delta_rot2_noise));
      delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(sqrt(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                       this->alpha2*delta_trans*delta_trans)));

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat *
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat *
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
    }
  }
  break;
  }
  return true;
}
