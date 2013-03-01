/*
 * Smooth.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_SMOOTH_H_
#define SKELTRACK_SMOOTH_H_

#include <cstddef>
#include <vector>
#include "SmoothData.h"

namespace Skeltrack {

class Smooth {
public:
	Smooth();
	virtual ~Smooth();

	static float holt_double_exp_formula_st(float alpha, float previous_trend, float current_value, float previous_smoothed_value);
	static float holt_double_exp_formula_bt(float beta, float previous_trend, float current_smoothed_value, float previous_smoothed_value);
	static void holt_double_exp_joint(float alpha, float beta, Joint *smoothed_joint, Joint *current_joint, Joint *trend_joint);

	void reset_joints_persistency_counter(SmoothData *smooth_data);
	void decrease_joints_persistency(SmoothData *smooth_data);
	void smooth_joints(SmoothData *data, std::vector<Joint *> * new_joints = NULL);
};

} /* namespace Skeltrack */
#endif /* SKELTRACK_SMOOTH_H_ */
