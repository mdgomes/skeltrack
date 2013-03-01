/*
 * SmoothData.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_SMOOTHDATA_H_
#define SKELTRACK_SMOOTHDATA_H_

#include <cstddef>
#include <vector>
#include "Joint.h"
#include "JointId.h"

namespace Skeltrack {

class SmoothData {
public:
	SmoothData(void) {
		joints_persistency = -1;
		smoothing_factor = -1;
		smoothed_joints = NULL;
		trend_joints = NULL;
	}
	virtual ~SmoothData(void){
		delete smoothed_joints;
		delete trend_joints;
	}

	void create_new_joint_list(void) { smoothed_joints = new std::vector<Joint *>(SKELTRACK_JOINT_MAX_JOINTS); }
	void create_new_trend_list(void) { trend_joints = new std::vector<Joint *>(SKELTRACK_JOINT_MAX_JOINTS); }

	std::vector<Joint *> * getSmoothedJoints(void) { return smoothed_joints; }
	std::vector<Joint *> * getTrendJoints(void) { return trend_joints; }

	Joint * getSmoothedJoint(int id) {
		if (smoothed_joints == NULL) return NULL;
		return smoothed_joints->at(id);
	}
	Joint * getTrendJoint(int id) {
		if (trend_joints == NULL) return NULL;
		return trend_joints->at(id);
	}

	void setSmoothedJoint(int id, Joint * joint) {
		if (smoothed_joints == NULL) create_new_joint_list();
		smoothed_joints->at(id) = joint;
	}
	void setTrendJoint(int id, Joint * joint) {
		if (trend_joints == NULL) create_new_trend_list();
		trend_joints->at(id) = joint;
	}

	unsigned int getJointsPersistency(void) { return joints_persistency; }
	void setJointsPersistency(unsigned int persistency) { joints_persistency = persistency; }
	float getSmoothingFactor(void) { return smoothing_factor; }
	void setSmoothingFactor(float factor) { smoothing_factor = factor; }

	void resetPersistencyCounter(void) {
		for (unsigned int i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
			joints_persistency_counter[i] = joints_persistency;
	}
	unsigned int getPersistencyCounter(unsigned int id) { return joints_persistency_counter[id]; }
	void decreasePersistencyCounter(unsigned int id) { joints_persistency_counter[id]--; }
	void setPersistencyCounter(unsigned int id, unsigned int counter) {
		joints_persistency_counter[id] = counter;
	}
protected:
	std::vector<Joint *> * smoothed_joints;
	std::vector<Joint *> * trend_joints;
	unsigned int joints_persistency;
	float smoothing_factor;
	unsigned int joints_persistency_counter[SKELTRACK_JOINT_MAX_JOINTS];
};

}

#endif /* SMOOTHDATA_H_ */
