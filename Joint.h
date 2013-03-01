/*
 * Joint.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_JOINT_H_
#define SKELTRACK_JOINT_H_

#include <string>
#include <cmath>
#include "JointId.h"

namespace Skeltrack {

#define SKELTRACK_JOINT_MAX_JOINTS 11

/**
 * SECTION:Joint
 * @short_description: Data structure that holds information about
 * a skeleton joint.
 *
 * A #SkeltrackJoint is built automatically by #SkeltrackSkeleton when
 * it finds a skeleton joint and can be used to get information about it.
 * Each #SkeltrackJoint holds an id, given by #SkeltrackJointId that indicates
 * which of the human skeleton joints it represents.
 *
 * Spacial information about a joint is given by the @x, @y and @z coordinates.
 * To represent the joint in a 2D, the variables @screen_x and
 * @screen_y will indicate the joint's position in the screen and are calculated
 * taking into account the #SkeltrackSkeleton:dimension-reduction (it will
 * be multiplied by this value).
 *
 * The tracked list of joints is represented by #SkeltrackJointList and given
 * by skeltrack_skeleton_track_joints_finish().
 * To get a #SkeltrackJoint from a #SkeltrackJointList object, use the
 * skeltrack_joint_list_get_joint() indicating the needed #SkeltrackJointId.
 *
 * A #SkeltrackJointList can be freed by using skeltrack_joint_list_free().
 * A #SkeltrackJoint can be copied by copy() and freed by skeltrack_joint_free().
 **/
class Joint {
public:
	/**
	 * Joint
	 * @param id: The id of the joint
	 * @param x: The x coordinate of the joint in the space (in mm)
	 * @param y: The y coordinate of the joint in the space (in mm)
	 * @param z: The z coordinate of the joint in the space (in mm)
	 * @param screen_x: The x coordinate of the joint in the screen (in pixels)
	 * @param screen_y: The y coordinate of the joint in the screen (in pixels)
	 * @param confidence: The confidence of the joint
	 **/
	Joint(Skeltrack::JointId id, int x = 0, int y = 0, int z = 0, int screen_x = 0, int screen_y = 0, float confidence = 1.0);
	virtual ~Joint();

	Skeltrack::JointId getId(void) { return id; }

	int getX(void) { return x; }
	void setX(int value) { x = value; }
	int getY(void) { return y; }
	void setY(int value) { y = value; }
	int getZ(void) { return z; }
	void setZ(int value) { z = value; }

	void setPosition(int x, int y, int z) { this->x = x; this->y = y; this->z = z; }

	int getScreenX(void) { return screen_x; }
	void setScreenX(int value) { screen_x = value; }
	int getScreenY(void) { return screen_y; }
	void setScreenY(int value) { screen_y = value; }

	void setScreenPosition(int x, int y) { screen_x = x; screen_y = y; }

	float getConfidence(void) { return confidence; }

	void setConfidence(float value) { if (value < 0 || value > 1) return; confidence = value; }

	/**
	 * Gets the distance to another joint in 3D space.
	 * @param other The other joint
	 * @return The distance in mm to the other joint
	 */
	double getDistanceToJoint(Joint * other) {
		double dx = (x - other->getX());
		double dy = (y - other->getY());
		double dz = (z - other->getZ());
		return sqrt(dx*dx+dy*dy+dz*dz);
	}

	/**
	 * Gets the angle between this and another joint.
	 * @param other The other joint
	 * @return The angle in radians between the joints
	 */
	double getAngleToJoint(Joint * other) {
		double lA = sqrt(x*x+y*y+z*z);
		double lB = sqrt(other->getX()*other->getX()+other->getY()*other->getY()+other->getZ()*other->getZ());
		double result = (x/lA)*(other->getX()/lB)+(y/lA)*(other->getY()/lB)+(z/lA)*(other->getZ()/lB);
		return acos(result);
	}

	Joint * copy(void);

	inline std::string resolveJointID(void) {
		switch(id) {
			case 0: return "HEAD";
			case 1: return "LEFT SHOULDER";
			case 2: return "RIGHT SHOULDER";
			case 3: return "SHOULDER CENTRE";
			case 4: return "LEFT ELBOW";
			case 5: return "RIGHT ELBOW";
			case 6: return "LEFT HAND";
			case 7: return "RIGHT HAND";
			case 8: return "CENTER";
			case 9: return "LEFT HIP";
			case 10: return "RIGHT HIP";
			default: return "UNKNOWN";
		}
	}

protected:
	Skeltrack::JointId id;
	int x;
	int y;
	int z;
	int screen_x;
	int screen_y;
	float confidence;
};

} /* namespace Skeltrack */
#endif /* JOINT_H_ */
