/*
 * Test.cpp
 *
 *  Created on: 13 de Fev de 2013
 *      Author: fahrenheit
 */

#include "Test.h"

namespace Skeltrack {

Test::Test() {}

Test::~Test() {}

void Test::loop(void) {
	for (int i = 0; i < NUMBER_OF_FILES; i++) {
		Fixture * fixture = new Fixture();
		fixture->skeleton = new Skeleton();
		std::cout << "[INFO] Skeltrack::Test::loop() test_track_joints_number()[" << i <<", "<<DEPTH_FILES[i]<<"]" << std::endl;
		test_track_joints_number(fixture,DEPTH_FILES[i]);
		std::cout << "[INFO] Skeltrack::Test::loop() track_joints_finish()[" << i <<"]" << std::endl;
		std::vector<Joint *> * joints = fixture->skeleton->track_joints_finish();
		int numberOfValidJoints = get_number_of_valid_joints(*joints);
		std::cout << "[INFO] Number of valid joints: " << numberOfValidJoints << std::endl;
		if (numberOfValidJoints > 0) {
			std::vector<Joint *>::iterator it;
			for (it = joints->begin(); it != joints->end(); it++) {
				Joint * joint = (*it);
				if (joint == NULL) continue;
				std::cout << "\t["<<joint->resolveJointID()<<"] ("<<joint->getX()<<", "<<joint->getY()<<", "<<joint->getZ()<<") ["<<joint->getScreenX()<<", "<<joint->getScreenY()<<"]"<<std::endl;
			}
		}
		delete fixture;
	}
}

} /* namespace Skeltrack */
