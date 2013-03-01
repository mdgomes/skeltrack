/*
 * SkeletonRepresentation.h
 *
 *  Created on: 26 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELETONREPRESENTATION_H_
#define SKELETONREPRESENTATION_H_

#include <cmath>
#include <list>
#include <vector>
#include <map>
#include <utility>
#include <string>
#include <iostream>
#include <limits>

#include "Skeleton.h"
#include "Joint.h"
#include "JointId.h"
#include "Util.h"

namespace Skeltrack {

typedef struct {
	double x;
	double y;
	double z;
} PointVector;

class BoneRepresentation {
public:
	/**
	 * Represents a bone.
	 * @param endpoint1 One of the bone endpoints
	 * @param endpoint2 The other bone endpoint
	 * @param distance An initial distance in mm (default: -1, use the addMeasure methods)
	 * @param acceptableDelta A new measure for this bone will only be accepted if
	 * 							the difference between the current difference between the
	 * 							stored distance and the new distance is bellow this value (default: 0.25)
	 */
	BoneRepresentation(JointId endpoint1, JointId endpoint2, int distance = -1, float acceptableDelta = 0.25) {
		this->endpoint1 = endpoint1;
		this->endpoint2 = endpoint2;
		this->distance = distance;
		this->acceptableDelta = acceptableDelta;
		angle = 0;
		numberOfmeasures = (distance >= 0 ? 1 : 0);
	}
	virtual ~BoneRepresentation(void) {};

	virtual void processJointPair(Joint * jointA, Joint * jointB) {
		if (jointA == NULL || jointB == NULL) return;
		// validate joints
		if (!(jointA->getId() == endpoint1 || jointA->getId() == endpoint2)) return;
		if (!(jointB->getId() == endpoint1 || jointB->getId() == endpoint2)) return;
		if (jointA->getId() == jointB->getId()) return;
		if (addMeasure((int)jointA->getDistanceToJoint(jointB)))
			angle = jointA->getAngleToJoint(jointB); // also set the angle
		//std::cout << "[Bone]("<<jointA->resolveJointID()<<"-"<<jointB->resolveJointID()<<") distance="<<this->distance<<", angle="<<(angle*(180.0/M_PI))<<std::endl;
	}

	virtual int getLength(void) { return distance; }

	virtual void setAcceptableDelta(float value) {
		if (value < 0 || value > 1) return;
		acceptableDelta = value;
	}

	virtual unsigned int getNumberOfMeasures(void) { return numberOfmeasures; }

	virtual float getAcceptableDelta(void) { return acceptableDelta; }

	virtual float getAngleBetweenJoints(void) { return angle; }

	virtual JointId getEndPoint1(void) { return endpoint1; }
	virtual JointId getEndPoint2(void) { return endpoint2; }
protected:
	JointId endpoint1;
	JointId endpoint2;
	int distance;
	float acceptableDelta;
	unsigned int numberOfmeasures;
	float angle;

	virtual bool addMeasure(int distance) {
		if (numberOfmeasures > 30) { // test for delta, only statistically relevant above 30 observations
			float delta = (this->distance > distance ? (float)distance/(float)this->distance : (float)this->distance/distance);
			if (1 - delta > acceptableDelta) return false; // ignore this measure
		}
		this->distance = (int)((float)(this->distance+distance)/2.0f);
		numberOfmeasures++;
		return true;
	}
};

class JointRepresentation {
public:
	JointRepresentation(JointId id, bool fastJoint = false) {
		this->id = id;
		this->fastJoint = fastJoint;
		maximumDeltaT = (fastJoint ? CLOCKS_PER_SEC/2 : CLOCKS_PER_SEC);
		expectedObservation = NULL;
		previousObservation = NULL;
		representatingJoint = NULL;
		lastTimestamp = 0;
		connectedJoints = NULL;
		connectedBones = NULL;
		connectedJointIds = NULL;
		deltaX = 0;
		deltaY = 0;
		deltaZ = 0;
		deltaT = 0;
		stableDelta = 0;
		boxTL = PointVector();
		boxTR = PointVector();
		boxBL = PointVector();
		boxBR = PointVector();
		boxC = PointVector();
		resetLocalBox();
		movementIsLocal = false;
	}

	virtual ~JointRepresentation(void) {
		if (previousObservation)
			delete previousObservation;
	}

	virtual void addConnection(JointRepresentation * joint, bool bidirectional = false) {
		if (!connectedJoints)
			connectedJoints = new std::vector<JointRepresentation *>();
		if (!connectedBones)
			connectedBones = new std::vector<BoneRepresentation *>();
		if (!connectedJointIds)
			connectedJointIds = new std::list<JointId>();
		if (getRepresentationId(joint->id) == -1) {
			connectedJoints->push_back(joint);
			connectedBones->push_back(new BoneRepresentation(this->id,joint->id));
			connectedJointIds->push_back(joint->getId());
		}
		if (bidirectional)
			joint->addConnection(this,false); // don't propagate
	}

	virtual void updateRepresentatingJoint(unsigned int timestamp) {
		if (lastTimestamp == 0) {
			representatingJoint = previousObservation ? previousObservation->copy() : NULL;
			return;
		}
		if (lastTimestamp == timestamp) {
			representatingJoint = previousObservation ? previousObservation->copy() : NULL;
			return;
		}
		unsigned int dt = timestamp - lastTimestamp;
		if (dt > maximumDeltaT) {
			representatingJoint = NULL; // no observations in the last second is too much error prone
			return;
		}
		int px = 0;
		int py = 0;
		int pz = 0;
		int eex = 0;
		int eey = 0;
		int eez = 0;
		if (previousObservation) {
			px = previousObservation->getX();
			py = previousObservation->getY();
			pz = previousObservation->getZ();
		}
		if (expectedObservation) {
			eex = px + (expectedObservation->x*dt);
			eey = py + (expectedObservation->y*dt);
			eez = pz + (expectedObservation->z*dt);
		} else {
			eex = px;
			eey = py;
			eez = pz;
		}
		Joint * newJoint = new Joint(id,eex,eey,eez);
		newJoint->setConfidence((float)dt/(float)maximumDeltaT); // to indicate that this was created by an heuristic
		representatingJoint = newJoint;
	}

	virtual void updateJointProperties(Joint * joint, unsigned int timestamp) {
		if (joint == NULL) return;
		if (lastTimestamp > 0) { // we can use some heuristics
			deltaX = previousObservation->getX() - joint->getX();
			deltaY = previousObservation->getY() - joint->getY();
			deltaZ = previousObservation->getZ() - joint->getZ();
			float ddx = 1.0;
			float ddy = 1.0;
			float ddz = 1.0;
			deltaT = timestamp - lastTimestamp;
			// we allow low X/Y movement and a bit more on Z
			if (abs(deltaX) < 25 && abs(deltaY) < 25 && abs(deltaZ) < 50) {
				// we have stable observation so add the current deltaT to the stable value
				stableDelta += deltaT;
			} else
				stableDelta = 0;
/*
			if (expectedObservation) {
				// calculate deviation from expected values
				int eex = previousObservation->getX() + (expectedObservation->getX()*dt);
				int eey = previousObservation->getY() + (expectedObservation->getY()*dt);
				int eez = previousObservation->getZ() + (expectedObservation->getZ()*dt);
				//std::cout<<"[JOINT]["<<joint->resolveJointID()<<"] expected Position("<<eex<<","<<eey<<","<<eez<<") vs observed ("<<joint->getX()<<","<<joint->getY()<<","<<joint->getZ()<<")"<<std::endl;

				ddx = deltaX != 0 ? (expectedObservation->getX()*deltaT)/deltaX : 0;
				ddy = deltaY != 0 ? (expectedObservation->getY()*deltaT)/deltaY : 0;
				ddz = deltaZ != 0 ? (expectedObservation->getZ()*deltaT)/deltaZ : 0;
			}
*/
			// interpolate new expected values and take into account deviation
			double ex, ey, ez;
			if (deltaT > 0) {
				ex = ((float)deltaX/(float)deltaT)*ddx;
				ey = ((float)deltaY/(float)deltaT)*ddy;
				ez = ((float)deltaZ/(float)deltaT)*ddz;
			} else {
				ex = 0;
				ey = 0;
				ez = 0;
			}
			if (!expectedObservation)
				expectedObservation = new PointVector();
			// update expected position so that we can use this to interpolate observations if needed
			expectedObservation->x = ex;
			expectedObservation->y = ey;
			expectedObservation->z = ez;

		}
		// now update previous observation
		previousObservation = joint->copy();
		lastTimestamp = timestamp;

		if (isStable()) // the joint is stable so we can get local movements
			updateLocalBox(); // update the box
		else
			validateLocalBox();

		// update representating joint
		updateRepresentatingJoint(timestamp);
	}

	virtual void processJointList(std::vector<Joint *> * joints, unsigned int timestamp) {
		if (joints == NULL) return;
		Joint * currentObservation = joints->at((unsigned int)id);
		if (currentObservation == NULL)
			currentObservation = getRepresentatingJoint();
		if (currentObservation == NULL) return; // nothing to do
		if (!(connectedJointIds == NULL || connectedJointIds->empty())) {
			for (std::list<JointId>::iterator it = connectedJointIds->begin(); it != connectedJointIds->end(); it++) {
				Joint * other = joints->at((unsigned int)(*it));
				int otherId = getRepresentationId((JointId)(*it));
				JointRepresentation * representation = connectedJoints->at(otherId);
				if (other == NULL)
					other = representation->getRepresentatingJoint();
				if (other == NULL) // we could not use the estimation because too much time has passed
					continue;
				BoneRepresentation * bone = connectedBones->at(otherId);
				bone->processJointPair(currentObservation,other);
			}
		}
	}

	virtual JointId getId(void) { return id; }

	virtual Joint * getRepresentatingJoint(void) { return representatingJoint; }

	virtual int getDeltaX(void) { return deltaX; }
	virtual bool isStableX(void) { return abs(deltaX) < 25; }
	virtual int getDeltaY(void) { return deltaY; }
	virtual bool isStableY(void) { return abs(deltaY) < 25; }
	virtual int getDeltaZ(void) { return deltaZ; }
	virtual bool isStableZ(void) { return abs(deltaZ) < 50; }
	virtual unsigned int getDeltaT(void) { return deltaT; }
	virtual unsigned int getStableDelta(void) { return stableDelta; }
	virtual bool isStable(void) { return stableDelta > maximumDeltaT; }

	virtual bool isMovementLocal(void) { return movementIsLocal; }

	virtual float getLocalX(void) {
		if (movementIsLocal) {
			float value = previousObservation->getX();
			float diff = abs(boxTL.x) + abs(boxBR.x);
			float x = (value - (float)boxTL.x) / diff;
			if (x < 0) x = 0;
			if (x > 1) x = 1;
			return x;
		}
		return -1;
	}
	virtual float getLocalY(void) {
		if (movementIsLocal) {
			float value = previousObservation->getY();
			float diff = abs(boxTL.y) + abs(boxBR.y);
			float y = (value - (float)boxBR.y) / diff;
			if (y < 0) y = 0;
			if (y > 1) y = 1;
			return y;
		}
		return -1;
	}
protected:
	JointId id;
	bool fastJoint;
	PointVector * expectedObservation;
	Joint * previousObservation;
	Joint * representatingJoint;
	unsigned int lastTimestamp;
	unsigned int maximumDeltaT;
	std::vector<JointRepresentation * > * connectedJoints;
	std::vector<BoneRepresentation *> * connectedBones;
	std::list<JointId> * connectedJointIds;

	int deltaX;
	int deltaY;
	int deltaZ;
	unsigned int deltaT;
	unsigned stableDelta;

	PointVector boxTL, boxTR, boxBL, boxBR, boxC;
	bool movementIsLocal;

	virtual int getRepresentationId(JointId id) {
		if (connectedJointIds == NULL || connectedJointIds->empty()) return -1;
		std::list<JointId>::iterator it;
		int idx = -1;
		for (it = connectedJointIds->begin(); it != connectedJointIds->end(); it++) {
			idx++;
			if ((*it) == id) return idx;
		}
		return -1; // not found
	}

	virtual void resetLocalBox(void) {
		boxTL.x = 0; boxTL.y = 0; boxTL.z = 0;
		boxTR.x = 0; boxTR.y = 0; boxTR.z = 0;
		boxBL.x = 0; boxBL.y = 0; boxBL.z = 0;
		boxBR.x = 0; boxBR.y = 0; boxBR.z = 0;
		boxC.x = 0 ; boxC.y = 0; boxC.z = 0;
		return; // nothing more to do
	}

	virtual void updateLocalBox(void) {
		if (stableDelta < maximumDeltaT && !(boxC.x == 0 && boxC.y == 0 && boxC.z == 0)) {
			resetLocalBox();
			return;
		}
		int dist = getShortestBoneLength(id);
		// TODO check if would be a good idea to use the screen aspect ratio or just a box
		//float ar = 1.0;
		int px = previousObservation->getX();
		int py = previousObservation->getY();
		int pz = previousObservation->getZ();
		boxTL.x = px - dist; boxTL.y = py + dist; boxTL.z = pz;
		boxTR.x = px + dist; boxTR.y = py + dist; boxTR.z = pz;
		boxBL.x = px - dist; boxBL.y = py - dist; boxBL.z = pz;
		boxBR.x = px + dist; boxBR.y = py - dist; boxBR.z = pz;
		boxC.x = px; boxC.y = py; boxC.z = pz;
//		std::cout << "[DEBUG]["<<previousObservation->resolveJointID()<<"] is stable and we created a new stable box with l="<<dist<<std::endl;
	}

	virtual void validateLocalBox(void) {
		if (boxC.x == 0 && boxC.y == 0 && boxC.z == 0) {
			movementIsLocal = false; // movement is global
			return;
		}
		int w = abs(boxTR.x) - abs(boxC.x);
		int h = abs(boxTL.y) - abs(boxC.y);
		int px = previousObservation->getX();
		int py = previousObservation->getY();
		if (px < boxTL.x || px > boxTR.x) { // check X bounds
			if (px < boxTL.x - w) { movementIsLocal = false; return; }
			if (px > boxTR.x + w) { movementIsLocal = false; return; }
		}
		if (py < boxBL.y || py > boxTL.y) { // check X bounds
			if (py < boxBL.y - h) { movementIsLocal = false; return; }
			if (py > boxTL.y + h) { movementIsLocal = false; return; }
		}
		movementIsLocal = true;
	}

	/**
	 * Gets the minimum length of all the bones connected to this joint.
	 * @param endpoint If set as different from the current joint Id specifies which bone we want
	 * @return shortest length or MAX_INT on some error
	 */
	virtual int getShortestBoneLength(JointId endpoint) {
		int shortestBoneLength = std::numeric_limits<int>::max();
		std::vector<BoneRepresentation *>::iterator it;
		for (it = connectedBones->begin(); it != connectedBones->end(); it++) {
			BoneRepresentation * bone = (*it);
			if (endpoint != id) { // check for specificity
				if (bone->getEndPoint1() == id && bone->getEndPoint2() != endpoint) continue;
				if (bone->getEndPoint2() == id && bone->getEndPoint1() != endpoint) continue;
			}
			int boneLength = bone->getLength();
			if (boneLength < shortestBoneLength)
				shortestBoneLength = boneLength;
		}
		return shortestBoneLength;
	}
};

class SkeletonRepresentation : public Skeltrack::Skeleton {
public:
	SkeletonRepresentation(void) : Skeltrack::Skeleton() {
		// create the skeleton representation
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_HEAD));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_LEFT_SHOULDER));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_SHOULDER_CENTER));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_RIGHT_SHOULDER));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_LEFT_ELBOW,true));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_RIGHT_ELBOW,true));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_LEFT_HAND,true));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_RIGHT_HAND,true));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_CENTER));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_LEFT_HIP));
		addJointToMap(new JointRepresentation(SKELTRACK_JOINT_ID_RIGHT_HIP));
		// add conections
		addConnection(SKELTRACK_JOINT_ID_HEAD,SKELTRACK_JOINT_ID_SHOULDER_CENTER);
		addConnection(SKELTRACK_JOINT_ID_LEFT_SHOULDER,SKELTRACK_JOINT_ID_SHOULDER_CENTER);
		addConnection(SKELTRACK_JOINT_ID_RIGHT_SHOULDER,SKELTRACK_JOINT_ID_SHOULDER_CENTER);
		addConnection(SKELTRACK_JOINT_ID_LEFT_SHOULDER,SKELTRACK_JOINT_ID_LEFT_ELBOW);
		addConnection(SKELTRACK_JOINT_ID_RIGHT_SHOULDER,SKELTRACK_JOINT_ID_RIGHT_ELBOW);
		addConnection(SKELTRACK_JOINT_ID_LEFT_ELBOW,SKELTRACK_JOINT_ID_LEFT_HAND);
		addConnection(SKELTRACK_JOINT_ID_RIGHT_ELBOW,SKELTRACK_JOINT_ID_RIGHT_HAND);
		addConnection(SKELTRACK_JOINT_ID_SHOULDER_CENTER,SKELTRACK_JOINT_ID_CENTER);
		addConnection(SKELTRACK_JOINT_ID_LEFT_HIP,SKELTRACK_JOINT_ID_CENTER);
		addConnection(SKELTRACK_JOINT_ID_RIGHT_HIP,SKELTRACK_JOINT_ID_CENTER);
	}
	virtual ~SkeletonRepresentation(void) {
		jointMap.clear();
	}

	virtual void updateRepresentation(unsigned int timestamp, bool updateJointList = true, bool onlyInterpolate = false) {
		if (!onlyInterpolate) {
			if (track_joints_result == NULL) return;
			std::map<JointId, JointRepresentation *>::iterator it;
			// first update joint properties
			for (it = jointMap.begin(); it != jointMap.end(); it++) {
				JointId id = it->first;
				JointRepresentation * jr = it->second;
				Joint * joint = track_joints_result->at((unsigned int)id);
				if (joint == NULL) continue;
				jr->updateJointProperties(joint,timestamp);
				// need to update some values
				if (jr->getRepresentatingJoint()->getConfidence() < 1) {
					Joint * newJoint = jr->getRepresentatingJoint();
					unsigned int si, sj;
					util.convert_mm_to_screen_coords(640,480,dimension_reduction,newJoint->getX(),newJoint->getY(),newJoint->getZ(),&si,&sj);
					newJoint->setScreenPosition(si,sj);
				}
			}
			// then update bones
			for (it = jointMap.begin(); it != jointMap.end(); it++) {
				JointRepresentation * jr = it->second;
				jr->processJointList(track_joints_result,timestamp);
			}
		} else {
			// we only wish to interpolate stuff
			std::map<JointId, JointRepresentation *>::iterator it;
			// first update joint properties
			for (it = jointMap.begin(); it != jointMap.end(); it++) {
				JointRepresentation * jr = it->second;
				jr->updateRepresentatingJoint(timestamp);
				// need to update some values
				Joint * newJoint = jr->getRepresentatingJoint();
				if (newJoint == NULL) continue;
				if (newJoint->getConfidence() < 1) {
					unsigned int si, sj;
					util.convert_mm_to_screen_coords(640,480,dimension_reduction,newJoint->getX(),newJoint->getY(),newJoint->getZ(),&si,&sj);
					newJoint->setScreenPosition(si,sj);
				}
			}
		}
		// if we want to update the internal joint list with our information
		if (updateJointList) {
			if (track_joints_result == NULL) {
				track_joints_result = new std::vector<Joint *>(jointMap.size());
			}
			for (unsigned int i = 0; i < track_joints_result->size(); i++) {
				Skeltrack::Joint * originalJoint = track_joints_result->at(i);
				if (!originalJoint) {
					// check if we have a representation for it
					JointRepresentation * jr = jointMap.at((JointId)i);

					Joint * newJoint = jr->getRepresentatingJoint();
					if (!newJoint) continue;
					unsigned int si, sj;
					util.convert_mm_to_screen_coords(640,480,dimension_reduction,newJoint->getX(),newJoint->getY(),newJoint->getZ(),&si,&sj);
					newJoint->setScreenPosition(si,sj);
					//std::cout << "[DEBUG] KinectWrapper::processBuffers() Updated representation of "<<newJoint->resolveJointID()<<" with ("<<newJoint->getX()<<", "<<newJoint->getY()<<", "<<newJoint->getZ()<<") ["<<si<<", "<<sj<<"] confidence="<<newJoint->getConfidence()<<std::endl;
					track_joints_result->at(i) = newJoint;
				}
			}
		}
	}

	virtual std::vector<JointRepresentation *> * getSkeletonRepresentation(unsigned int timestamp) {
		std::vector<JointRepresentation *> * newJoints = new std::vector<JointRepresentation *>(SKELTRACK_JOINT_MAX_JOINTS);
		for (unsigned int i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++) {
			JointRepresentation * representation = jointMap.at((JointId)i);
			newJoints->at(i) = representation;
		}
		return newJoints;
	}
protected:
	std::map<JointId,JointRepresentation *> jointMap;

	virtual void addJointToMap(JointRepresentation * jr) {
		jointMap.insert(std::pair<JointId,JointRepresentation *>(jr->getId(),jr));
	}

	virtual void addConnection(JointId endpoint1, JointId endpoint2, bool propagate = true) {
		JointRepresentation * j1 = jointMap.at(endpoint1);
		JointRepresentation * j2 = jointMap.at(endpoint2);
		j1->addConnection(j2,propagate);
	}
};

} /* namespace CSIVega */
#endif /* SKELETONREPRESENTATION_H_ */
