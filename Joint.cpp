/*
 * Joint.cpp
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#include "Joint.h"

namespace Skeltrack {
/**
 * Joint
 * @param id: The id of the joint
 * @param x: The x coordinate of the joint in the space (in mm)
 * @param y: The y coordinate of the joint in the space (in mm)
 * @param z: The z coordinate of the joint in the space (in mm)
 * @param screen_x: The x coordinate of the joint in the screen (in pixels)
 * @param screen_y: The y coordinate of the joint in the screen (in pixels)
 * @param confidence: Confidence in the measure (1 is full confidence, 0 is no confidence)
 **/
Joint::Joint(Skeltrack::JointId id, int x, int y, int z, int screen_x, int screen_y, float confidence) {
	this->id = id;
	this->x = x;
	this->y = y;
	this->z = z;
	this->screen_x = screen_x;
	this->screen_y = screen_y;
	this->confidence = confidence;
}

/**
 * skeltrack_joint_free:
 * @joint: The #SkeltrackJoint to free
 *
 * Frees a #SkeltrackJoint object.
 **/
Joint::~Joint() {}

/**
 * skeltrack_joint_copy:
 * @joint: The #SkeltrackJoint to copy
 *
 * Makes an exact copy of a #SkeltrackJoint object.
 *
 * Returns: (transfer full): A newly created #SkeltrackJoint. Use
 * skeltrack_joint_free() to free it.
 **/
Joint * Joint::copy(void) {
	return new Joint(this->getId(),this->x,this->y,this->z,this->screen_x,this->screen_y);
}

} /* namespace Skeltrack */


