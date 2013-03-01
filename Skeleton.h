/*
 * Skeleton.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_SKELETON_H_
#define SKELTRACK_SKELETON_H_

#include <cstddef>
#include <iostream>
#include <list>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "Smooth.h"
#include "Util.h"
#include "Node.h"
#include "Label.h"
#include "Joint.h"
#include "SkeletonProperties.h"
#include "../Mutex.h"

namespace Skeltrack {
/**
 * SECTION:Skeleton
 * @short_description: Object that tracks the joints in a human skeleton
 *
 * This object tries to detect joints of the human skeleton.
 *
 * To track the joints, first create an instance of #SkeltrackSkeleton using
 * skeltrack_skeleton_new() and then set a buffer from where the joints will
 * be retrieved using the asynchronous function
 * skeltrack_skeleton_track_joints() and get the list of joints using
 * skeltrack_skeleton_track_joints_finish().
 *
 * A common use case is to use this library together with a Kinect device so
 * an easy way to retrieve the needed buffer is to use the GFreenect library.
 *
 * It currently tracks the joints identified by #JointId .
 *
 * Tracking the skeleton joints can be computational heavy so it is advised that
 * the given buffer's dimension is reduced before setting it. To do it,
 * simply choose the reduction factor and loop through the original buffer
 * (using this factor as a step) and set the reduced buffer's values accordingly.
 * The #SkeltrackSkeleton:dimension-reduction property holds this reduction
 * value and should be changed to the reduction factor used (alternatively you
 * can retrieve its default value and use it in the reduction, if it fits your
 * needs).
 *
 * The skeleton tracking uses a few heuristics that proved to work well for
 * tested cases but they can be tweaked by changing the following properties:
 * #Skeleton:graph-distance-threshold ,
 * #Skeleton:graph-minimum-number-nodes ,
 * #Skeleton:hands-minimum-distance ,
 * #Skeleton:shoulders-arc-start-point ,
 * #Skeleton:shoulders-arc-length ,
 * #Skeleton:shoulders-circumference-radius ,
 * #Skeleton:shoulders-search-step .
 **/
class Skeleton {
public:
	Skeleton(void);
	virtual ~Skeleton(void);

	void track_joints(unsigned short * buffer, unsigned int width, unsigned int height);
	std::vector<Joint *> * track_joints_finish(void);
	std::vector<Joint *> * track_joints_sync(unsigned short * buffer, unsigned int width, unsigned int height);

	void get_focus_point(int * x, int * y, int * z);
	void set_focus_point(int x, int y, int z);

	void setProperty(SkeletonProperties prop, float value);
	float getProperty(SkeletonProperties prop);

	Util * getUtil(void) { return &util; }
protected:
	Util util;
	Smooth smooth;

	std::vector<Joint *> joints;

	unsigned short *buffer;
	unsigned int buffer_width;
	unsigned int buffer_height;

	std::vector<Joint *> * track_joints_result;
	CSIVega::Mutex * track_joints_mutex;

	std::vector<Node *> * graph;
	std::vector<Label *> *labels;
	std::vector<Node *> * node_matrix;
	int  *distances_matrix;
	std::vector<Node *> * main_component;

	unsigned int dimension_reduction;
	unsigned int distance_threshold;
	unsigned int min_nr_nodes;

	unsigned int hands_minimum_distance;

	unsigned int shoulders_circumference_radius;
	unsigned int shoulders_arc_start_point;
	unsigned int shoulders_arc_length;
	float shoulders_search_step;
	float chest_factor;

	unsigned int extrema_sphere_radius;

	Node * focus_node;

	bool enable_smoothing;
	SmoothData smooth_data;

	float torso_minimum_number_nodes;

	Joint * previous_head;

	int join_neighbor(Node *node, std::vector<Label *> *neighbor_labels, int index, int i, int j);
	Node * get_centroid(void);
	Node * get_lowest(Node *centroid);
	Node * get_longer_distance(int * distances);
	void set_average_extremas(std::vector<Node *> * extremas);
	std::vector<Node *> * get_extremas(Node *centroid);

	std::vector<Node *> * make_graph(std::vector<Label *> * label_list);

	Node * get_shoulder_node(float alpha, float step, int x_node, int y_node, int z_centroid);
	bool check_if_node_can_be_head(Node *node, Node *centroid, Node ** left_shoulder, Node ** right_shoulder);
	bool get_head_and_shoulders(std::vector<Node *> *extremas, Node *centroid, Node ** head, Node ** left_shoulder, Node ** right_shoulder);
	void identify_arm_extrema(int *distances, std::vector<Node*>* previous_nodes, int width, int hand_distance, Node *extrema, Node ** elbow_extrema, Node ** hand_extrema);
	void set_left_and_right_from_extremas(std::vector<Node *> *extremas, Node *head, Node *left_shoulder, Node *right_shoulder, std::vector<Joint *> *joints);
	Node * get_adjusted_shoulder (unsigned int buffer_width, unsigned int buffer_height, unsigned int dimension_reduction, std::vector<Node *> *graph, Node *centroid, Node *head, Node *shoulder);
	Node * get_shoulder_center(unsigned int buffer_width, unsigned int buffer_height, unsigned int dimension_reduction, std::vector<Node *> * graph, Node * left_shoulder, Node * right_shoulder, Node * head);
	std::vector<Joint *> * _track_joints(void);

	void track_joints_in_thread(void);

	void clean_tracking_resources(void);
};

} /* namespace Skeltrack */
#endif /* SKELTRACK_SKELETON_H_ */
