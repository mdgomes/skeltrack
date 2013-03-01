/*
 * Util.cpp
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#include "Util.h"

namespace Skeltrack {

Util::Util(float scale_factor, float min_distance) {
	this->scale_factor = scale_factor;
	this->min_distance = min_distance;
}

Util::~Util() {}

// Static Methods
Joint * Util::node_to_joint(Node * node, JointId id, int dimension_reduction) {
	Joint * joint;

	if (node == NULL)
		return NULL;

	joint = new Joint(id,node->getX(), node->getY(), node->getZ(), node->getI() * dimension_reduction, node->getJ() * dimension_reduction);

	return joint;
}

unsigned int Util::get_distance_from_joint(Node *node, Joint *joint)
{
	double dx, dy, dz;
	dx = (double)abs(node->getX() - joint->getX());
	dy = (double)abs(node->getY() - joint->getY());
	dz = (double)abs(node->getZ() - joint->getZ());
	return (unsigned int)sqrt(dx * dx + dy * dy + dz * dz);
}

void Util::unlink_node(Node * node) {
	Node *neighbour;
	std::list<Node *>::iterator it;
	std::list<Node *> * node_neighbours = node->getNeighbours();
	if (node_neighbours == NULL) return; // nothing to do
	// first remove the node from all it's neighbours list
	for (it = node_neighbours->begin(); it != node_neighbours->end(); it++) {
		neighbour = (*it);
		neighbour->removeNodeFromNeighbours(node);
		neighbour->removeNodeFromLinkedNodes(node);
	}
}

/**
 * Returns the closest node of a given node.
 * @param node_list Nodes to compare the distance
 * @param from Node which to calculate the distance
 * @param x_dist Required x distance
 * @param y_dist Required y distance
 * @param z_dist Required z distance
 * @param closest_node_dist Pointer to the distance of the closest node (-1 if there is no closest node)
 * @return Closest node or NULL if there is no such node
 */
Node * Util::get_closest_node_with_distances (std::vector<Node *> node_list, Node *from, unsigned int x_dist, unsigned int y_dist, unsigned int z_dist, int *closest_node_dist) {
	/* @TODO: Replace this and use closest pair of points
	     algorithm and ensure O(n log n) instead of brute-force */
	std::vector<Node *>::iterator it;
	Node *closest = NULL;
	int distance = -1;

	for (it = node_list.begin(); it != node_list.end(); it++)
	{
		unsigned int dx, dy, dz;
		Node *node = (*it);
		int current_distance;

		dx = abs(from->getX() - node->getX());
		dy = abs(from->getY() - node->getY());
		dz = abs(from->getZ() - node->getZ());

		if (dx > x_dist || dy > y_dist || dz > z_dist)
			continue;

		current_distance = sqrt (dx * dx + dy * dy + dz * dz);
		if (closest == NULL || distance > current_distance)
		{
			closest = node;
			distance = current_distance;
		}
	}

	*closest_node_dist = distance;
	return closest;
}

Node * Util::get_closest_node_to_joint(std::vector<Node *> extremas, Joint * joint, int * distance) {
	int dist = -1;
	Node *closest_node = NULL;
	std::vector<Node *>::iterator it;
	for (it = extremas.begin(); it != extremas.end(); it++) {
		unsigned int current_dist;
		Node *node = (*it);
		if (node == NULL)
			continue;
		current_dist = get_distance_from_joint (node, joint);
		if (dist == -1 || current_dist < (unsigned int)dist)
		{
			closest_node = node;
			dist = current_dist;
		}
	}
	*distance = dist;
	return closest_node;
}

Node * Util::get_closest_node(std::vector<Node *> node_list, Node *from) {
	Node *closest = NULL;
	int distance = -1;
	std::vector<Node *>::iterator it;

	/* @TODO: Replace this and use closest pair of points
	     algorithm and ensure O(n log n) instead of brute-force */

	for (it = node_list.begin(); it != node_list.end(); it++)
	{
		Node *node = (*it);
		int current_distance;
		if (closest == NULL)
		{
			closest = node;
			distance = get_distance (node, from);
			continue;
		}
		current_distance = get_distance (node, from);
		if (current_distance < distance)
		{
			closest = node;
			distance = current_distance;
		}
	}
	return closest;
}

Node * Util::get_closest_torso_node(std::vector<Node *> * node_list, Node *from, Node * head) {
	Node *closest = NULL;
	int distance = -1;
	std::vector<Node *>::iterator it;

	/* @TODO: Replace this and use closest pair of points
	     algorithm and ensure O(n log n) instead of brute-force */

	for (it = node_list->begin(); it != node_list->end(); it++)
	{
		Node *node = (*it);
		int current_distance;
		if (node->getZ() >= head->getZ() && node->getY() >= from->getY())
		{
			current_distance = get_distance (node, from);
			if (closest == NULL || current_distance < distance)
			{
				closest = node;
				distance = current_distance;
			}
		}
	}
	return closest;
}

Label * Util::get_main_component(std::vector<Node *> * node_list, Node *from, double min_normalized_nr_nodes) {
	Label *main_component = NULL;
	if (node_list == NULL) return NULL;
	int distance = -1;
	std::vector<Node *>::iterator it;

	for (it = node_list->begin(); it != node_list->end(); it++)
	{
		Node *node = (*it);
		Label *label;
		int current_distance;
		label = node->getLabel();

		if (main_component == NULL && label->getNormalizedNumNodes() > min_normalized_nr_nodes)
		{
			main_component = label;
			distance = get_distance (node, from);
			continue;
		}

		current_distance = get_distance (node, from);
		if (current_distance < distance &&
				label->getNormalizedNumNodes() > min_normalized_nr_nodes)
		{
			main_component = label;
			distance = current_distance;
		}
	}

	return main_component;
}

Label * Util::label_find(Label * label) {
	Label *parent;

	if (label == NULL) return NULL;

	parent = label->getParent();
	if (parent == label)
		return parent;
	else
		return label_find(parent);
}

void Util::label_union(Label * a, Label * b) {
	Label *root_a, *root_b;
	root_a = label_find (a);
	root_b = label_find (b);
	if (root_a->getIndex() < root_b->getIndex())
	{
		b->setParent(root_a);
	}
	else
	{
		a->setParent(root_b);
	}
}

double Util::get_distance(Node *a, Node *b) {
	return a->distanceToNode(b);
}

void Util::free_label(Label * label) {
	delete label;
}

void Util::clean_labels(std::vector<Label *> * labels) {
	if (labels == NULL) return; // nothing to do
	std::vector<Label *>::iterator it = labels->begin();
	while (it != labels->end()) {
		delete (*it);
		it++;
	}
}

void Util::free_node(Node * node, bool unlink_first) {
	if (unlink_first)
	{
		unlink_node (node);
	}
	node = NULL;
}

void Util::clean_nodes(std::vector<Node *> * nodes) {
	if (nodes == NULL) return; // nothing to do
	std::vector<Node *>::iterator it = nodes->begin();
	while (it != nodes->end()) {
		free_node((*it),false);
		it++;
	}
}

std::vector<Node *> * Util::remove_nodes_with_label(std::vector<Node *> * nodes, std::vector<Node *> * node_matrix, int width, Label * label) {
	if (nodes == NULL) return nodes; // nothing to do, so just return
	Node * node = NULL;
	std::vector<Node *>::iterator nit = nodes->begin();
	Node * current_node = NULL;

	current_node = nit != nodes->end() ? (*nit) : NULL;
	while (current_node != NULL)
	{
		node = current_node;
		if (node->getLabel() == label)
		{
			free_node (node, true); // free the node first
			nit = nodes->erase(nit); // no need to do ++ as this will point to the next valid node
			current_node = (nit != nodes->end() ? (*nit) : NULL);
			node_matrix->at(width * node->getJ() + node->getI()) = NULL;
			continue;
		}
		if (current_node != NULL) { // only increment if not already NULL
			nit++;
			current_node = (nit != nodes->end() ? (*nit) : NULL);
		}
	}
	return nodes;
}

Label * Util::get_lowest_index_label(std::vector<Label *>* neighbor_labels) {
	unsigned int index;
	Label *lowest_index_label = NULL;

	lowest_index_label = neighbor_labels->at(0);
	for (index = 1; index < 4; index++)
	{
		if (neighbor_labels->at(index) == NULL)
			continue;

		if (lowest_index_label == NULL ||
				lowest_index_label->getIndex() < neighbor_labels->at(index)->getIndex())
		{
			lowest_index_label = neighbor_labels->at(index);
		}
	}

	return lowest_index_label;
}

Label * Util::new_label(int index) {
	return new Label(index);
}

void Util::join_components_to_main(std::vector<Label *> labels, Label *main_component_label, unsigned int horizontal_max_distance, unsigned int depth_max_distance, unsigned int graph_distance_threshold) {
	Label *current_label;
	std::vector<Label*>::iterator it = labels.begin();

	for (current_label = labels.front(); current_label != NULL;)
	{
		int closer_distance = -1;
		Label *label;
		Node *current_node;

		label = current_label;
		if (label == main_component_label) {
			it++;
			current_label = it != labels.end() ? (*it) : NULL;
			continue;
		}

		/* Skip nodes behind main component */
		if (label->getHigherZ() > main_component_label->getHigherZ() + (int)graph_distance_threshold) {
			it++;
			current_label = it != labels.end() ? (*it) : NULL;
			continue;
		}

		std::vector<Node *>::iterator nit;
		if (label->getNodes() != NULL)
			nit = label->getNodes()->begin();
		for (current_node = (label->getNodes() != NULL && !label->getNodes()->empty() ? label->getNodes()->front() : NULL);
				current_node != NULL;)
		{
			Node *node;
			int current_distance;
			node = current_node;
			/* Skip nodes that belong to the same component or
	             that a not in the edge of their component */
			unsigned int sz = node->getNeighbours() != NULL && !node->getNeighbours()->empty() ? node->getNeighbours()->size() : 0;
			if (sz == 8) {
				nit++;
				current_node = nit != label->getNodes()->end() ? (*nit) : NULL;
				continue;
			}

			Node *closest_node = get_closest_node_with_distances (*main_component_label->getNodes(),
							node,
							horizontal_max_distance,
							horizontal_max_distance,
							depth_max_distance,
							&current_distance);
			if (closest_node && (current_distance < closer_distance || closer_distance == -1))
			{
				node->getLabel()->setBridgeNode(node);
				node->getLabel()->setToNode(closest_node);
				closer_distance = current_distance;
			}
			nit++;
			current_node = nit != label->getNodes()->end() ? (*nit) : NULL;
		}

		it++;
		current_label = it != labels.end() ? (*it) : NULL;
	}
}

void Util::set_joint_from_node(std::vector<Joint *> * joints, Node *node, JointId id, int dimension_reduction) {
	joints->at(id) = node_to_joint(node, id, dimension_reduction);
}

int * Util::create_new_dist_matrix(int matrix_size) {
	int i;
	int *distances;

	distances = (int *)malloc(matrix_size * sizeof (int));
	for (i = 0; i < matrix_size; i++)
	{
		distances[i] = -1;
	}
	return distances;
}

bool Util::dijkstra_to(std::vector<Node *> nodes, Node *source, Node *target, int width, int height, int *distances, std::vector<Node *> * previous) {
	PQueue *queue = new PQueue(nodes.size(), width, height);

	std::vector<Node *>::iterator it;
	for (it = nodes.begin(); it != nodes.end(); it++) {
		Node * node = (*it);
		if (previous != NULL)
			previous->at(node->getJ() * width + node->getI()) = NULL;

		if (node == source)
			queue->insert(node,0);
		else
			queue->insert(node,std::numeric_limits<int>::max());
	}

	distances[source->getJ() * width + source->getI()] = 0;

	std::list<Node *>::iterator cit;
	Node * current_neighbor = NULL;
	while (!queue->empty())
	{
		Node *node = queue->pop_minimum();

		if (target != NULL && node == target)
		{
			delete queue;
			return true;
		}

		if (distances [node->getJ() * width + node->getI()] == -1)
			continue;

		if (node->getNeighbours() != NULL) {
			cit = node->getNeighbours()->begin();
			current_neighbor = (*cit);
		}
		while (current_neighbor)
		{
			unsigned int dist = 0;
			Node *neighbor = current_neighbor;

			if (queue->has_element(neighbor))
			{
				dist = get_distance (node, neighbor) + distances[node->getJ() * width + node->getI()];
				queue->remove(neighbor);

				if (distances[neighbor->getJ() * width + neighbor->getI()] == -1 ||
						(distances[neighbor->getJ() * width + neighbor->getI()] != -1 &&
								dist < distances[neighbor->getJ() * width + neighbor->getI()]))
				{
					distances[neighbor->getJ() * width + neighbor->getI()] = dist;
					if (previous)
						previous->at(neighbor->getJ() * width + neighbor->getI()) = node;
				}

				queue->insert(neighbor, distances[neighbor->getJ() * width + neighbor->getI()]);
			}

			cit++;
			current_neighbor = (cit != node->getNeighbours()->end() ? (*cit) : NULL);
		}

	}
	delete queue;
	return false;
/*
	int nr;
	std::list<Node *> * unvisited_nodes = new std::list<Node *>();
	std::list<Node *>::iterator vit;
	std::vector<Node *>::iterator it, nit, rit;
	it = nodes.begin();

	Node * current = NULL;

	for (current = nodes.front();
			previous != NULL && current != NULL && it != nodes.end();)
	{
		Node *node = current;
		previous->at(node->getJ() * width + node->getI()) = NULL;
		it++;
		current = it != nodes.end() ? (*it) : NULL;
	}
	distances[source->getJ() * width + source->getI()] = 0;

	unvisited_nodes->assign(nodes.begin(),nodes.end());

	nr = 0;
	Node * unvisited_node = unvisited_nodes->front();
	vit = unvisited_nodes->begin();
	std::list<Node *>::iterator shorter_dist_node_it = unvisited_nodes->begin();
	while (unvisited_node != NULL)
	{
		Node *node;
		Node *current_neighbor, *shorter_dist_node, *cur_node;

		shorter_dist_node = unvisited_node;
		vit = unvisited_nodes->begin();
		shorter_dist_node_it = vit;
		vit++;
		cur_node = vit != unvisited_nodes->end() ? (*vit) : NULL;
		while (cur_node != NULL)
		{
			Node *value, *shorter_dist;
			value = cur_node;
			shorter_dist = shorter_dist_node;
			if (distances[shorter_dist->getJ() * width + shorter_dist->getI()] == -1 ||
					(distances[value->getJ() * width + value->getI()] != -1 &&
							distances[value->getJ() * width +
							          value->getI()] < distances[shorter_dist->getJ() * width +
							                                shorter_dist->getI()]))
			{
				shorter_dist_node = cur_node;
				shorter_dist_node_it = vit;
			}
			vit++;
			cur_node = vit != unvisited_nodes->end() ? (*vit) : NULL;
		}

		node = shorter_dist_node;
		if (distances[node->getJ() * width + node->getI()] == -1)
		{
			break;
		}

		current_neighbor = node->getNeighbours() != NULL && !node->getNeighbours()->empty() ? node->getNeighbours()->front() : NULL;
		std::vector<Node *>::iterator vnit;
		if (node->getNeighbours() != NULL) vnit = node->getNeighbours()->begin();
		while (current_neighbor)
		{
			int dist;
			Node *neighbor = current_neighbor;
			dist = get_distance(node, neighbor) + distances[node->getJ() * width + node->getI()];

			if (distances[neighbor->getJ() * width + neighbor->getI()] == -1 ||
					dist < distances[neighbor->getJ() * width + neighbor->getI()])
			{
				// printf("[Neighbour][%d,%d]\n",neighbor->j,neighbor->i);
				distances[neighbor->getJ() * width + neighbor->getI()] = dist;

				if (previous != NULL)
				{
					previous->at(neighbor->getJ() * width + neighbor->getI()) = node;
				}
				nr++;
			}
			if (target != NULL && neighbor == target)
			{
				delete unvisited_nodes;
				return true;
			}

			vnit++;
			current_neighbor = vnit != node->getNeighbours()->end() ? (*vnit) : NULL;
		}
		unvisited_nodes->erase(shorter_dist_node_it);
		unvisited_node = !unvisited_nodes->empty() ? unvisited_nodes->front() : NULL;
		//unvisited_nodes = g_list_delete_link (unvisited_nodes, shorter_dist_node);
	}

	delete unvisited_nodes;
	return false;
*/
}

void Util::convert_screen_coords_to_mm(unsigned int width, unsigned int height, unsigned int dimension_reduction, unsigned int i, unsigned int j, int  z, int *x, int *y) {
	float width_height_relation =
			width > height ? (float) width / height : (float) height / width;
	/* Formula from http://openkinect.org/wiki/Imaging_Information */
	*x = round((i * dimension_reduction - width * dimension_reduction / 2.0) * (z + min_distance) * scale_factor * width_height_relation);
	*y = round((j * dimension_reduction - height * dimension_reduction / 2.0) * (z + min_distance) * scale_factor);
}

void Util::convert_mm_to_screen_coords(unsigned int width, unsigned int height, unsigned int dimension_reduction, int x, int y, int z, unsigned int *i, unsigned int *j) {
	float width_height_relation = width > height ? (float) width / height : (float) height / width;

	if (z + min_distance == 0) {
		*i = 0;
		*j = 0;
		return;
	}

	*i = round (width / 2.0 + x / ((float) (z + min_distance) * scale_factor * dimension_reduction * width_height_relation));
	*j = round (height / 2.0 + y / ((float) (z + min_distance) * scale_factor * dimension_reduction));
}

float Util::get_angle_between_nodes(Node * a, Node *b) {
	float sin_angle, opp, slope;
	opp = (float) a->getX() - b->getX();
	slope = sqrt (pow (a->getX() - b->getX(), 2) + pow ((a->getY() - b->getY()), 2));
	if (slope == 0)
		return 0;
	sin_angle = opp / slope;
	return asin (-sin_angle);
}

} /* namespace Skeltrack */
