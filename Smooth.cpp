/*
 * Smooth.cpp
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#include "Smooth.h"

namespace Skeltrack {

Smooth::Smooth() {}

Smooth::~Smooth() {}

float Smooth::holt_double_exp_formula_st(float alpha, float previous_trend, float current_value, float previous_smoothed_value)
{
  return alpha * current_value + (1.0 - alpha) * (previous_smoothed_value + previous_trend);
}

float Smooth::holt_double_exp_formula_bt (float beta, float previous_trend, float current_smoothed_value, float previous_smoothed_value)
{
  return beta * (current_smoothed_value - previous_smoothed_value) + (1.0 - beta) * previous_trend;
}

void Smooth::holt_double_exp_joint (float alpha, float beta, Joint *smoothed_joint, Joint *current_joint, Joint *trend_joint)
{
  float new_x, new_y, new_z, new_screen_x, new_screen_y;
  new_x = holt_double_exp_formula_st (alpha,trend_joint->getX(),current_joint->getX(),smoothed_joint->getX());
  new_y = holt_double_exp_formula_st (alpha,trend_joint->getY(),current_joint->getY(),smoothed_joint->getY());
  new_z = holt_double_exp_formula_st (alpha,trend_joint->getZ(),current_joint->getZ(),smoothed_joint->getZ());
  new_screen_x = holt_double_exp_formula_st (alpha,trend_joint->getScreenX(),current_joint->getScreenX(),smoothed_joint->getScreenX());
  new_screen_y = holt_double_exp_formula_st (alpha,trend_joint->getScreenY(),current_joint->getScreenY(),smoothed_joint->getScreenY());
  trend_joint->setX(holt_double_exp_formula_bt (beta,trend_joint->getX(),new_x,smoothed_joint->getX()));
  trend_joint->setY(holt_double_exp_formula_bt (beta,trend_joint->getY(),new_y,smoothed_joint->getY()));
  trend_joint->setZ(holt_double_exp_formula_bt (beta,trend_joint->getZ(),new_z,smoothed_joint->getZ()));
  trend_joint->setScreenX(holt_double_exp_formula_bt (beta,trend_joint->getScreenX(),new_screen_x,smoothed_joint->getScreenX()));
  trend_joint->setScreenY(holt_double_exp_formula_bt (beta,trend_joint->getScreenY(),new_screen_y,smoothed_joint->getScreenY()));
  smoothed_joint->setX(new_x);
  smoothed_joint->setY(new_y);
  smoothed_joint->setZ(new_z);
  smoothed_joint->setScreenX(new_screen_x);
  smoothed_joint->setScreenY(new_screen_y);
}

void Smooth::reset_joints_persistency_counter(SmoothData *smooth_data) {
	unsigned int i;
	for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
	{
		smooth_data->setPersistencyCounter(i,smooth_data->getJointsPersistency());
	}
}

void Smooth::decrease_joints_persistency(SmoothData *smooth_data)
{
	unsigned int i;
	for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
	{
		Joint *smoothed_joint = NULL;
		Joint *trend_joint = NULL;

		if (smooth_data->getSmoothedJoints() != NULL)
			smoothed_joint = smooth_data->getSmoothedJoints()->at(i);
		if (smooth_data->getTrendJoints() != NULL)
			trend_joint = smooth_data->getTrendJoints()->at(i);

		if (smoothed_joint != NULL || trend_joint != NULL)
		{
			if (smooth_data->getPersistencyCounter(i) > 0)
				smooth_data->decreasePersistencyCounter(i);
			else
			{
				if (smoothed_joint) smooth_data->setSmoothedJoint(i,NULL);
				if (trend_joint) smooth_data->setTrendJoint(i, NULL);
				delete smoothed_joint;
				delete trend_joint;
				smooth_data->setPersistencyCounter(i,smooth_data->getJointsPersistency());
			}
		}
	}
}

void Smooth::smooth_joints(SmoothData *data, std::vector<Joint *> * new_joints) {
	unsigned int i;

	if (new_joints == NULL)
	{
		decrease_joints_persistency (data);
		return;
	}

	if (data->getSmoothedJoints() == NULL)
	{
		data->create_new_joint_list();
		for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
		{
			Joint * joint = new_joints->at(i);
			data->setSmoothedJoint(i,joint != NULL ? joint->copy() : NULL);
		}
		return;
	}
	if (data->getTrendJoints() == NULL)
	{
		data->create_new_trend_list();
	}

	for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
	{
		Joint *joint, *smoothed_joint, *trend_joint;

		smoothed_joint = data->getSmoothedJoint(i);
		trend_joint = data->getTrendJoint(i);
		joint = new_joints->at(i);
		if (joint == NULL)
		{
			if (smoothed_joint != NULL)
			{
				if (data->getPersistencyCounter(i) > 0)
					data->decreasePersistencyCounter(i);
				else
				{
					data->setSmoothedJoint(i,NULL);
					data->setTrendJoint(i,NULL);
					delete smoothed_joint;
					delete trend_joint;
					data->setPersistencyCounter(i,data->getJointsPersistency());
				}
			}
			continue;
		}
		data->setPersistencyCounter(i,data->getJointsPersistency());

		if (smoothed_joint == NULL)
		{
			data->setSmoothedJoint(i,joint->copy());
			continue;
		}

		if (trend_joint == NULL)
		{
			/* First case (when there are only initial values) */
			trend_joint = new Joint((JointId)i);
			trend_joint->setX(joint->getX() - smoothed_joint->getX());
			trend_joint->setY(joint->getY() - smoothed_joint->getY());
			trend_joint->setZ(joint->getZ() - smoothed_joint->getZ());
			trend_joint->setScreenX(joint->getScreenX() - smoothed_joint->getScreenX());
			trend_joint->setScreenY(joint->getScreenY() - smoothed_joint->getScreenY());
			data->setTrendJoint(i,trend_joint);
		}
		else
		{
			/* @TODO: Check if we should give the control of each factor
	             independently (data-smoothing-factor and trend-smoothing-factor).
			 */
			holt_double_exp_joint (
					data->getSmoothingFactor(),
					data->getSmoothingFactor(),
					smoothed_joint, joint,
					trend_joint);
		}
	}
}

} /* namespace Skeltrack */
