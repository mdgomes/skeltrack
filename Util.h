/*
 * Util.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_UTIL_H_
#define SKELTRACK_UTIL_H_

#include <cstddef>
#include <list>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>

#include "JointId.h"
#include "Joint.h"
#include "Label.h"
#include "Node.h"
#include "PQueue.h"

namespace Skeltrack {

class PQueueCompare {
	bool reverse;
public:
	PQueueCompare(const bool& revparam=false) { reverse = revparam; }
	bool operator()(const Node * lhs, const Node * rhs) {
		if(reverse) return (lhs > rhs);
		else return (rhs > lhs);
	}
};

class Util {
public:
	Util(float scale_factor = 0.0021, float min_distance = -10.0);
	virtual ~Util();

	float getScaleFactor(void) { return scale_factor; }
	void setScaleFactor(float value) { scale_factor = value; }
	float getMinDistance(void) { return min_distance; }
	void setMinDistance(float value) { min_distance = value; }

	// Static methods
	Joint * node_to_joint(Node * node, JointId id, int dimension_reduction);
	unsigned int get_distance_from_joint(Node *node, Joint *joint);
	void unlink_node(Node *node);
	Node * get_closest_node_with_distances (std::vector<Node *> node_list, Node *from, unsigned int x_dist, unsigned int y_dist, unsigned int z_dist, int *closest_node_dist);

	// non static methods
	Node * get_closest_node_to_joint(std::vector<Node *> extremas, Joint * joint, int * distance);
	Node * get_closest_node(std::vector<Node *> node_list, Node *from);
	Node * get_closest_torso_node(std::vector<Node *> * node_list, Node *from, Node * head);
	Label * get_main_component(std::vector<Node *> * node_list, Node *from, double  min_normalized_nr_nodes);
	Label * label_find(Label * label);
	void label_union(Label * a, Label * b);
	double get_distance(Node *a, Node *b);
	float get_angle_between_nodes (Node *a, Node *b);
	void free_label(Label * label);
	void clean_labels(std::vector<Label *> * labels);
	void free_node(Node * node, bool unlink_first);
	void clean_nodes(std::vector<Node *> * nodes);
	std::vector<Node *> * remove_nodes_with_label(std::vector<Node *> * nodes, std::vector<Node *> * node_matrix, int width, Label * label);
	Label * get_lowest_index_label(std::vector<Label *> * neighbor_labels);
	Label * new_label(int index);
	void join_components_to_main(std::vector<Label *> nodes, Label *lowest_component_label, unsigned int horizontal_max_distance, unsigned int depth_max_distance, unsigned int graph_distance_threshold);
	void set_joint_from_node(std::vector<Joint *> * joints, Node *node, JointId id, int dimension_reduction);
	int * create_new_dist_matrix(int matrix_size);
	bool dijkstra_to(std::vector<Node *> nodes, Node *source, Node *target, int width, int height, int *distances, std::vector<Node *> * previous);
	void convert_screen_coords_to_mm(unsigned int width, unsigned int height, unsigned int dimension_reduction, unsigned int i, unsigned int j, int  z, int *x, int *y);
	void convert_mm_to_screen_coords(unsigned int width, unsigned int height, unsigned int dimension_reduction, int x, int y, int z, unsigned int *i, unsigned int *j);

protected:
	float scale_factor;
	float min_distance;
};

} /* namespace Skeltrack */
#endif /* UTIL_H_ */
