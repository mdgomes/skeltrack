/*
 * Skeleton.cpp
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#include "Skeleton.h"

namespace Skeltrack {

#define DIMENSION_REDUCTION 16
#define GRAPH_DISTANCE_THRESHOLD 150
#define GRAPH_MINIMUM_NUMBER_OF_NODES 5
#define HANDS_MINIMUM_DISTANCE 550
#define SHOULDERS_CIRCUMFERENCE_RADIUS 300
#define SHOULDERS_ARC_START_POINT 100
#define SHOULDERS_ARC_LENGTH 250
#define SHOULDERS_SEARCH_STEP 0.05
#define JOINTS_PERSISTENCY_DEFAULT 3
#define SMOOTHING_FACTOR_DEFAULT .5
#define ENABLE_SMOOTHING_DEFAULT true
#define DEFAULT_FOCUS_POINT_Z 1000
#define TORSO_MINIMUM_NUMBER_NODES_DEFAULT 16.0
#define EXTREMA_SPHERE_RADIUS 300
#define CHEST_FACTOR 2.75

/* Currently searches for head and hands */
static const unsigned int NR_EXTREMAS_TO_SEARCH  = 3;

Skeleton::Skeleton() {
	buffer = NULL;
	buffer_width = 0;
	buffer_height = 0;

	graph = NULL;
	labels = NULL;
	main_component = NULL;
	node_matrix = NULL;
	distances_matrix = NULL;

	dimension_reduction = DIMENSION_REDUCTION;
	distance_threshold = GRAPH_DISTANCE_THRESHOLD;

	min_nr_nodes = GRAPH_MINIMUM_NUMBER_OF_NODES;

	hands_minimum_distance = HANDS_MINIMUM_DISTANCE;

	shoulders_circumference_radius = SHOULDERS_CIRCUMFERENCE_RADIUS;
	shoulders_arc_start_point = SHOULDERS_ARC_START_POINT;
	shoulders_arc_length = SHOULDERS_ARC_LENGTH;
	shoulders_search_step = SHOULDERS_SEARCH_STEP;
	chest_factor = CHEST_FACTOR;

	extrema_sphere_radius = EXTREMA_SPHERE_RADIUS;

	focus_node = new Node(0,0,0,0,DEFAULT_FOCUS_POINT_Z);

	track_joints_result = NULL;

	track_joints_mutex = new CSIVega::Mutex();

	enable_smoothing = ENABLE_SMOOTHING_DEFAULT;
	smooth_data.setSmoothingFactor(SMOOTHING_FACTOR_DEFAULT);
	smooth_data.setJointsPersistency(JOINTS_PERSISTENCY_DEFAULT);
	smooth_data.resetPersistencyCounter();
	torso_minimum_number_nodes = TORSO_MINIMUM_NUMBER_NODES_DEFAULT;

	previous_head = NULL;
}

Skeleton::~Skeleton() {
	// clear joints
	joints.clear();
	if (track_joints_result != NULL) {
		track_joints_result->clear();
	}
	track_joints_mutex->unlock(); // unlock
	delete track_joints_mutex;
	if (graph != NULL) {
		graph->clear();
	}
	if (labels != NULL) {
		labels->clear();
	}
	if (node_matrix != NULL) {
		node_matrix->clear();
	}
	if (distances_matrix != NULL)
		free(distances_matrix);
	if (main_component != NULL) {
		main_component->clear();
	}
	if (focus_node)
		delete focus_node;
	if (previous_head)
		delete previous_head;
	if (buffer)
		free(buffer);
}

void Skeleton::setProperty(SkeletonProperties prop, float value) {
	switch (prop)
	{
	case PROP_DIMENSION_REDUCTION:
		dimension_reduction = (unsigned int)value;
		break;

	case PROP_GRAPH_DISTANCE_THRESHOLD:
		distance_threshold = (unsigned int)value;
		break;

	case PROP_GRAPH_MIN_NR_NODES:
		min_nr_nodes = (unsigned int)value;
		break;

	case PROP_HANDS_MINIMUM_DISTANCE:
		hands_minimum_distance = (unsigned int)value;
		break;

	case PROP_SHOULDERS_CIRCUMFERENCE_RADIUS:
		shoulders_circumference_radius = (unsigned int)value;
		break;

	case PROP_SHOULDERS_ARC_START_POINT:
		shoulders_arc_start_point = (unsigned int)value;
		break;

	case PROP_SHOULDERS_ARC_LENGTH:
		shoulders_arc_length = (unsigned int)value;
		break;

	case PROP_SHOULDERS_SEARCH_STEP:
		shoulders_search_step = value;
		break;

	case PROP_EXTREMA_SPHERE_RADIUS:
		extrema_sphere_radius = (unsigned int)value;
		break;

	case PROP_SMOOTHING_FACTOR:
		smooth_data.setSmoothingFactor(value);
		break;

	case PROP_JOINTS_PERSISTENCY:
		smooth_data.setJointsPersistency((unsigned int)value);
		smooth_data.resetPersistencyCounter();
		break;

	case PROP_ENABLE_SMOOTHING:
		enable_smoothing = (bool) value;
		break;

	case PROP_TORSO_MINIMUM_NUMBER_NODES:
		torso_minimum_number_nodes = value;
		break;

	default:
		std::cerr << "[WARN] Skeleton.setProperty(" << prop <<", "<<value<<") Invalid Property ID." << std::endl;
		break;
	}
}

float Skeleton::getProperty(SkeletonProperties prop) {
	switch (prop)
	{
	case PROP_DIMENSION_REDUCTION:
		return (float) dimension_reduction;
		break;

	case PROP_GRAPH_DISTANCE_THRESHOLD:
		return (float) distance_threshold;
		break;

	case PROP_GRAPH_MIN_NR_NODES:
		return (float) min_nr_nodes;
		break;

	case PROP_HANDS_MINIMUM_DISTANCE:
		return (float) hands_minimum_distance;
		break;

	case PROP_SHOULDERS_CIRCUMFERENCE_RADIUS:
		return (float) shoulders_circumference_radius;
		break;

	case PROP_SHOULDERS_ARC_START_POINT:
		return (float) shoulders_arc_start_point;
		break;

	case PROP_SHOULDERS_ARC_LENGTH:
		return (float) shoulders_arc_length;
		break;

	case PROP_SHOULDERS_SEARCH_STEP:
		return shoulders_search_step;
		break;

	case PROP_EXTREMA_SPHERE_RADIUS:
		return (float) extrema_sphere_radius;
		break;

	case PROP_SMOOTHING_FACTOR:
		return smooth_data.getSmoothingFactor();
		break;

	case PROP_JOINTS_PERSISTENCY:
		return (float) smooth_data.getJointsPersistency();
		break;

	case PROP_ENABLE_SMOOTHING:
		return (float) enable_smoothing;
		break;

	case PROP_TORSO_MINIMUM_NUMBER_NODES:
		return torso_minimum_number_nodes;
		break;

	default:
		std::cerr << "[WARN] Skeleton.getProperty(" << prop <<") Invalid Property ID." << std::endl;
		break;
	}
	return -1;
}

int Skeleton::join_neighbor(Node *node, std::vector<Label *> * neighbor_labels, int index, int i, int j)
{
	Node *neighbor;
	if (i < 0 || i >= (int)buffer_width || j < 0 || j >= (int)buffer_height) {
		return index;
	}

	neighbor = node_matrix->at(buffer_width * j + i);
	if (neighbor != NULL)
	{
		unsigned int distance = neighbor->distanceToNode(node);
		if (distance < distance_threshold)
		{
			neighbor->appendNeighbour(node);
			node->appendNeighbour(neighbor);
			neighbor_labels->at(index) = neighbor->getLabel();
			index++;
		}
	}
	return index;
}

Node * Skeleton::get_centroid(void)
{
	int avg_x = 0;
	int avg_y = 0;
	int avg_z = 0;
	int length = 0;
	std::vector<Node *> node_list;
	std::vector<Node *>::iterator it;
	Node *cent = NULL;
	Node *centroid = NULL;

	if (main_component == NULL)
		return NULL;

	for (it = main_component->begin(); it != main_component->end(); it++)
	{
		Node * node = (*it);
		avg_x += node->getX();
		avg_y += node->getY();
		avg_z += node->getZ();
	}

	length = main_component->size();
	cent = new Node(0,0);
	cent->setX(avg_x / length);
	cent->setY(avg_y / length);
	cent->setZ(avg_z / length);

	centroid = util.get_closest_node(*graph, cent);

	delete cent;

	return centroid;
}

Node * Skeleton::get_lowest(Node *centroid)
{
	Node *lowest = NULL;
	/* @TODO: Use the node_matrix instead of the lowest
     component to look for the lowest node as it's faster. */
	if (main_component != NULL)
	{
		std::vector<Node *>::iterator it;
		for (it = main_component->begin(); it != main_component->end(); it++) {
			Node *node = (*it);
			if (node->getI() != centroid->getI()) continue;
			if (lowest == NULL || lowest->getJ() < node->getJ()) lowest = node;
		}
	}
	return lowest;
}

Node * Skeleton::get_longer_distance(int * distances)
{
	if (graph == NULL) return NULL;
	Node *current = NULL;
	Node *farthest_node = NULL;

	std::vector<Node *>::iterator it = graph->begin();

	current = graph->front();
	farthest_node = current;
	it++;
	current = (*it);

	while (current != NULL)
	{
		Node *node = current;
		if (node != NULL && (distances[farthest_node->getJ() * buffer_width + farthest_node->getI()] != -1 &&
						distances[farthest_node->getJ() * buffer_width + farthest_node->getI()] <
						distances[node->getJ() * buffer_width + node->getI()]))
		{
			farthest_node = node;
		}
		it++;
		current = (it != graph->end() ? (*it) : NULL);
	}
	return farthest_node;
}

void Skeleton::set_average_extremas(std::vector<Node *> * extremas)
{
	if (extremas == NULL || extremas->empty()) return; // nothing to do
	Node * current_extrema;
	std::vector<Node *>::iterator cit = extremas->begin();
	std::vector<Node *> * averaged_extremas = NULL;

	unsigned int nr = 0;
	for (current_extrema = extremas->front(); current_extrema != NULL;)
	{
		std::vector<Node *>::iterator current_node;
		Node *extrema, *node = NULL, *cent = NULL, *node_centroid = NULL;
		int avg_x = 0, avg_y = 0, avg_z = 0, length = 0;

		extrema = current_extrema;

		for (current_node = graph->begin();
				current_node != graph->end();
				current_node++)
		{
			node = (*current_node);

			if ((util.get_distance(extrema, node) < extrema_sphere_radius))
			{
				avg_x += node->getX();
				avg_y += node->getY();
				avg_z += node->getZ();

				length++;
			}
		}

		/* if the length is 1 then it is because no other
	         nodes were considered for the average */
		if (length > 1)
		{
			cent = new Node(0,0);
			cent->setX(avg_x / length);
			cent->setY(avg_y / length);
			cent->setZ(avg_z / length);

			node_centroid = util.get_closest_node(*graph, cent);

			std::vector<Node *>::iterator loc;
			bool f = false;
			if (averaged_extremas != NULL) {
				for (loc = averaged_extremas->begin(); loc != averaged_extremas->end(); loc++) {
					if ((*loc) == node_centroid) { f = true; break; }
				}
			}
			if (!f) {
				for (loc = extremas->begin(); loc != extremas->end(); loc++) {
					if ((*loc) == node_centroid) { f = true; break; }
				}
			}
			if (!f) {
				extremas->at(nr) = node_centroid;
				current_extrema = node_centroid;
			}

			delete cent;
		}
		cit++;
		nr++;
		current_extrema = cit != extremas->end() ? (*cit) : NULL;
	}
}

std::vector<Node *> * Skeleton::get_extremas(Node *centroid)
{
	int i, nr_nodes, matrix_size;
	Node *lowest, *source, *node;
	std::vector<Node *> * extremas = NULL;

	lowest = get_lowest (centroid);
	source = lowest;

	matrix_size = buffer_width * buffer_height;
	if (distances_matrix == NULL)
	{
		distances_matrix = (int *)malloc(matrix_size * sizeof (int));
	}

	for (i = 0; i < matrix_size; i++)
	{
		distances_matrix[i] = -1;
	}

	for (nr_nodes = NR_EXTREMAS_TO_SEARCH; source != NULL && nr_nodes > 0; nr_nodes--)
	{
		util.dijkstra_to(*graph, source, NULL, buffer_width, buffer_height, distances_matrix, NULL);
		node = get_longer_distance (distances_matrix);

		if (node == NULL)
			continue;

		if (node != source)
		{
			distances_matrix[node->getJ() * buffer_width + node->getI()] = 0;
			source->appendLinkedNode(node);
			node->appendLinkedNode(source);
			source = node;
			if (extremas == NULL) extremas = new std::vector<Node *>();
			extremas->push_back(node);
		}
	}

	if (extrema_sphere_radius != 0)
	{
		set_average_extremas (extremas);
	}

	return extremas;
}

std::vector<Node *> * Skeleton::make_graph(std::vector<Label *> * label_list)
{
	int i, j;
	unsigned int n;
	Node *node;
	std::vector<Node *> * nodes = new std::vector<Node *>();
	std::vector<Label *> labels;
	std::vector<Label *>::iterator current_label;
	Label *main_component_label = NULL;
	int index = 0;
	int next_label = -1;
	unsigned short value;
	unsigned short * buffer;
	int width, height;

	buffer = this->buffer;
	width = buffer_width;
	height = buffer_height;

	if (node_matrix == NULL) {
		node_matrix = new std::vector<Node *>(width * height);
	} else {
		node_matrix->clear();
		// need to re-initialize
		for (i = 0; i < (width*height); i++)
			node_matrix->push_back(NULL);
	}

	for (i = 0; i < width; i++)
	{
		for (j = 0; j < height; j++)
		{
			int south, north, west;
			Label *lowest_index_label = NULL;
			std::vector<Label *> neighbor_labels(4);
			neighbor_labels.at(0) = NULL;
			neighbor_labels.at(1) = NULL;
			neighbor_labels.at(2) = NULL;
			neighbor_labels.at(3) = NULL;

			value = buffer[j * width + i];
			if (value == 0)
				continue;

			node = new Node(0,0);
			node->setI(i);
			node->setJ(j);
			node->setZ(value);
			int x,y;
			util.convert_screen_coords_to_mm (
					buffer_width,
					buffer_height,
					dimension_reduction,
					i, j,
					node->getZ(),
					&x,
					&y);
			node->setX(x);
			node->setY(y);

			index = 0;

			south = j + 1;
			north = j - 1;
			west = i - 1;

			/* West */
			index = join_neighbor(node, &neighbor_labels, index, west, j);
			/* South West*/
			index = join_neighbor(node, &neighbor_labels, index, west, south);
			/* North */
			index = join_neighbor(node, &neighbor_labels, index, i, north);
			/* North West */
			index = join_neighbor(node, &neighbor_labels, index, west, north);

			lowest_index_label = util.get_lowest_index_label (&neighbor_labels);

			/* No neighbors */
			if (lowest_index_label == NULL)
			{
				Label *label;
				next_label++;
				label = new Label(next_label);
				labels.push_back(label);
				lowest_index_label = label;
			}
			else
			{
				for (index = 0; index < 4; index++)
				{
					if (neighbor_labels[index] != NULL)
					{
						util.label_union (neighbor_labels[index], lowest_index_label);
					}
				}
			}

			node->setLabel(lowest_index_label);
			nodes->push_back(node);
			node_matrix->at(width * node->getJ() + node->getI()) = node;
		}
	}

	for (n = 0; n < nodes->size(); n++)
	{
		Node * node = nodes->at(n);
		node->setLabel(util.label_find(node->getLabel()));
		node->getLabel()->appendNode(node);

		/* Assign lower node so we can extract the
         lower graph's component */
		if (node->getLabel()->getLowerScreenY() == -1 || node->getJ() > node->getLabel()->getLowerScreenY())
		{
			node->getLabel()->setLowerScreenY(node->getJ());
		}

		/* Assign farther to the camera node so we
         can extract the main graph component */
		if (node->getLabel()->getHigherZ() == -1 || node->getZ() > node->getLabel()->getHigherZ())
		{
			node->getLabel()->setHigherZ(node->getZ());
		}

		/* Assign closer to the camera node so we
         can extract the main graph component */
		if (node->getLabel()->getLowerZ() == -1 || node->getZ() < node->getLabel()->getLowerZ())
		{
			node->getLabel()->setLowerZ(node->getZ());
		}
	}

	for (current_label = labels.begin(); current_label != labels.end(); current_label++)
	{
		Label *label = (*current_label);
		std::vector<Node *> * current_nodes = label->getNodes();
		unsigned int sz = current_nodes != NULL ? current_nodes->size() : 0;
		label->setNormalizedNumNodes(sz *
				((label->getHigherZ() - label->getLowerZ())/2 +
						label->getLowerZ()) *
						(pow (DIMENSION_REDUCTION, 2)/2) /
						1000000);
	}

	main_component_label = util.get_main_component(nodes, focus_node, torso_minimum_number_nodes);

	Label * c_label = !labels.empty() ? labels.front() : NULL;
	std::list<Label *> links_to_remove;
	std::vector<Label *>::iterator cit = labels.begin();
	while (c_label != NULL)
	{
		Label *label = c_label;

		/* Remove label if number of nodes is less than
	         the minimum required */
		unsigned int sz = label->getNodes() != NULL ? label->getNodes()->size() : 0;
		if (sz < min_nr_nodes)
		{
			nodes = util.remove_nodes_with_label(nodes, node_matrix, buffer_width, label);

			links_to_remove.push_back(c_label);
		}

		cit++;
		c_label = cit != labels.end() ? (*cit) : NULL;
	}
	// clear labels
	if (!links_to_remove.empty()) {
		for (std::list<Label *>::iterator r = links_to_remove.begin(); r != links_to_remove.end(); r++) {
			for (std::vector<Label *>::iterator v = labels.begin(); v != labels.end(); v++) {
				if ((*v) == (*r)) {
					labels.erase(v);
					break;
				}
			}
			util.free_label(*r);
		}
		links_to_remove.clear();
	}

	if (main_component_label)
	{
		util.join_components_to_main (labels,
				main_component_label,
				distance_threshold,
				hands_minimum_distance,
				distance_threshold);

		current_label = labels.begin();
		while (current_label != labels.end())
		{
			Label *label = (*current_label);
			if (label == main_component_label)
			{
				current_label++;
				continue;
			}

			if (label->getBridgeNode() == NULL)
			{
				nodes = util.remove_nodes_with_label (nodes, node_matrix, buffer_width, label);
				links_to_remove.push_back(*current_label);
				current_label++;
				continue;
			}
			label->getBridgeNode()->appendNeighbour(label->getToNode());
			label->getToNode()->appendNeighbour(label->getBridgeNode());

			current_label++;
		}

		main_component = main_component_label->getNodes();
	}
	// clear labels
	if (!links_to_remove.empty()) {
		for (std::list<Label *>::iterator r = links_to_remove.begin(); r != links_to_remove.end(); r++) {
			for (std::vector<Label *>::iterator v = labels.begin(); v != labels.end(); v++) {
				if ((*v) == (*r)) {
					labels.erase(v);
					break;
				}
			}
			util.free_label(*r);
		}
		links_to_remove.clear();
	}

	if (label_list == NULL)
		label_list = new std::vector<Label *>(labels.size());

	label_list->assign(labels.begin(),labels.end());

	return nodes;
}

Node * Skeleton::get_shoulder_node(
		float alpha,
		float step,
		int x_node,
		int y_node,
		int z_centroid)
{
	unsigned int radius, arc_start_point, arc_length, current_i, current_j;
	float start_angle, last_node_arc, current_arc, angle, current_x, current_y;
	Node *current_node = NULL;
	Node *last_node = NULL;

	radius = shoulders_circumference_radius;
	arc_start_point = shoulders_arc_start_point;
	arc_length = shoulders_arc_length;

	start_angle = M_PI_2;

	angle = start_angle + alpha;
	current_x = x_node + radius * cos (angle);
	current_y = y_node + radius * sin (angle);
	current_arc = 0;
	last_node_arc = 0;
	current_node = NULL;
	last_node = NULL;

	while (current_arc <= (arc_start_point + arc_length))
	{
		util.convert_mm_to_screen_coords (
				buffer_width,
				buffer_height,
				dimension_reduction,
				current_x,
				current_y,
				z_centroid,
				&current_i,
				&current_j);

		if (current_i >= buffer_width || current_j >= buffer_height)
			break;

		current_node = node_matrix->at(current_j * buffer_width + current_i);

		if (current_node != NULL)
		{
			last_node = current_node;
			last_node_arc = current_arc;
		}

		angle += step;
		current_x = x_node + radius * cos (angle);
		current_y = y_node + radius * sin (angle);
		current_arc = fabs(angle - start_angle) * (float)radius;
	}

	if (last_node_arc < arc_start_point)
		return NULL;

	return last_node;
}

bool Skeleton::check_if_node_can_be_head(Node *node, Node *centroid, Node ** left_shoulder, Node ** right_shoulder) {
	float alpha;
	Node *found_right_shoulder = NULL, *found_left_shoulder = NULL;

	*left_shoulder = NULL;
	*right_shoulder = NULL;

	if (node->getJ() > centroid->getJ())
		return false;

	if ((node->getY() - centroid->getY()) != 0)
		alpha = atan( fabs (node->getX() - centroid->getX()) / fabs (node->getY() - centroid->getY()));
	else
		return false;

	/* too much tilt, cannot be the head */
	if (alpha >= M_PI_4)
		return false;

	if (node->getX() < centroid->getX())
		alpha = -alpha;

	found_right_shoulder = get_shoulder_node (alpha, shoulders_search_step, node->getX(), node->getY(), centroid->getZ());
	if (found_right_shoulder == NULL)
		return false;

	found_left_shoulder = get_shoulder_node (alpha, -shoulders_search_step, node->getX(), node->getY(), centroid->getZ());

	if (found_left_shoulder == NULL)
		return false;

	*right_shoulder = found_right_shoulder;
	*left_shoulder = found_left_shoulder;

	return true;
}

bool Skeleton::get_head_and_shoulders(std::vector<Node *> *extremas, Node *centroid, Node ** head, Node ** left_shoulder, Node ** right_shoulder)
{
	Node *node;
	Node *current_extrema;
	std::vector<Node *>::iterator it;
	if (extremas == NULL) return false;
	it = extremas->begin();

	for (current_extrema = extremas->front(); current_extrema != NULL;)
	{
		node = current_extrema;
		if (check_if_node_can_be_head (node, centroid, left_shoulder, right_shoulder))
		{
			*head = node;
			return true;
		}
		it++;
		current_extrema = (it != extremas->end() ? (*it) : NULL);
	}
	return false;
}

void Skeleton::identify_arm_extrema(int *distances,
		std::vector<Node*> *previous_nodes,
		int width,
		int hand_distance,
		Node *extrema,
		Node ** elbow_extrema,
		Node ** hand_extrema)
{
	int total_dist;

	if (extrema == NULL)
		return;

	total_dist = distances[width * extrema->getJ() + extrema->getI()];
	if (total_dist < hand_distance)
	{
		*elbow_extrema = extrema;
		*hand_extrema = NULL;
	}
	else
	{
		Node *previous;
		int elbow_dist;

		previous = previous_nodes->at(extrema->getJ() * width + extrema->getI());
		elbow_dist = total_dist / 2;
		while (previous &&
				distances[previous->getJ() * width + previous->getI()] > elbow_dist)
		{
			previous = previous_nodes->at(previous->getJ() * width + previous->getI());
		}
		*elbow_extrema = previous;
		*hand_extrema = extrema;
	}
}

void Skeleton::set_left_and_right_from_extremas(std::vector<Node *> *extremas,
		Node *head,
		Node *left_shoulder,
		Node *right_shoulder,
		std::vector<Joint *> *joints)
{
	int *dist_left_a = NULL;
	int *dist_left_b = NULL;
	int *dist_right_a = NULL;
	int *dist_right_b = NULL;
	int total_dist_left_a = -1;
	int total_dist_right_a = -1;
	int total_dist_left_b = -1;
	int total_dist_right_b = -1;
	int *distances_left[2] = {NULL, NULL};
	int *distances_right[2] = {NULL, NULL};
	int index_left = -1;
	int index_right = -1;
	Node *elbow_extrema, *hand_extrema;
	std::vector<Node *> * previous_left_a = NULL;
	std::vector<Node *> * previous_left_b = NULL;
	std::vector<Node *> * previous_right_a = NULL;
	std::vector<Node *> * previous_right_b = NULL;
	std::vector<Node *> * previous_left[2] = {NULL, NULL};
	std::vector<Node *> * previous_right[2] = {NULL, NULL};
	Node *ext_a = NULL;
	Node *ext_b = NULL;
	Node *left_extrema[2] = {NULL, NULL};
	Node *right_extrema[2] = {NULL, NULL};
	std::vector<Node *>::iterator current_extrema;
	int width, height, matrix_size;

	for (current_extrema = extremas->begin();
			current_extrema != extremas->end();
			current_extrema++)
	{
		Node *node = (*current_extrema);
		if (node != head)
		{
			if (ext_a == NULL)
				ext_a = node;
			else
				ext_b = node;
		}
	}

	if (head == NULL)
		return;

	width = buffer_width;
	height = buffer_height;
	matrix_size = width * height;

	previous_left_a = new std::vector<Node *>(matrix_size);
	previous_left_b = new std::vector<Node *>(matrix_size);
	previous_right_a = new std::vector<Node *>(matrix_size);
	previous_right_b = new std::vector<Node *>(matrix_size);

	dist_left_a = util.create_new_dist_matrix(matrix_size);
	util.dijkstra_to(*graph,left_shoulder,ext_a,width,height,dist_left_a,previous_left_a);

	dist_left_b = util.create_new_dist_matrix(matrix_size);
	util.dijkstra_to(*graph,left_shoulder,ext_b,width,height,dist_left_b,previous_left_b);

	dist_right_a = util.create_new_dist_matrix(matrix_size);
	util.dijkstra_to(*graph,right_shoulder,ext_a,width,height,dist_right_a,previous_right_a);

	dist_right_b = util.create_new_dist_matrix(matrix_size);
	util.dijkstra_to(*graph,right_shoulder,ext_b,width,height,dist_right_b,previous_right_b);

	total_dist_left_a = dist_left_a[ext_a->getJ() * width + ext_a->getI()];
	total_dist_right_a = dist_right_a[ext_a->getJ() * width + ext_a->getI()];
	total_dist_left_b = dist_left_b[ext_b->getJ() * width + ext_b->getI()];
	total_dist_right_b = dist_right_b[ext_b->getJ() * width + ext_b->getI()];

	if (total_dist_left_a < total_dist_right_a)
	{
		index_left++;
		left_extrema[index_left] = ext_a;
		distances_left[index_left] = dist_left_a;
		previous_left[index_left] = previous_left_a;
	}
	else
	{
		index_right++;
		right_extrema[index_right] = ext_a;
		distances_right[index_right] = dist_right_a;
		previous_right[index_right] = previous_right_a;
	}

	if (total_dist_left_b < total_dist_right_b)
	{
		index_left++;
		left_extrema[index_left] = ext_b;
		distances_left[index_left] = dist_left_b;
		previous_left[index_left] = previous_left_b;
	}
	else
	{
		index_right++;
		right_extrema[index_right] = ext_b;
		distances_right[index_right] = dist_right_b;
		previous_right[index_right] = previous_right_b;
	}

	elbow_extrema = NULL;
	hand_extrema = NULL;
	identify_arm_extrema (distances_left[0],previous_left[0],width,hands_minimum_distance,left_extrema[0],&elbow_extrema,&hand_extrema);

	/* Two left extremas */
	if (index_left == 1)
	{
		if (hand_extrema == NULL)
		{
			hand_extrema = left_extrema[1];
			elbow_extrema = left_extrema[0];
		}
		else
		{
			hand_extrema = left_extrema[0];
			elbow_extrema = left_extrema[1];
		}
	}

	util.set_joint_from_node(joints,elbow_extrema,SKELTRACK_JOINT_ID_LEFT_ELBOW,dimension_reduction);
	util.set_joint_from_node(joints,hand_extrema,SKELTRACK_JOINT_ID_LEFT_HAND,dimension_reduction);


	elbow_extrema = NULL;
	hand_extrema = NULL;
	identify_arm_extrema(distances_right[0],previous_right[0],width,hands_minimum_distance,right_extrema[0],&elbow_extrema,&hand_extrema);

	/* Two right extremas */
	if (index_right == 1)
	{
		if (hand_extrema == NULL)
		{
			hand_extrema = right_extrema[1];
			elbow_extrema = right_extrema[0];
		}
		else
		{
			hand_extrema = right_extrema[0];
			elbow_extrema = right_extrema[1];
		}
	}

	util.set_joint_from_node(joints,elbow_extrema,SKELTRACK_JOINT_ID_RIGHT_ELBOW,dimension_reduction);
	util.set_joint_from_node(joints,hand_extrema,SKELTRACK_JOINT_ID_RIGHT_HAND,	dimension_reduction);

	delete previous_left_a;
	delete previous_left_b;
	delete previous_right_a;
	delete previous_right_b;

	free(dist_left_a);
	free(dist_left_b);
	free(dist_right_a);
	free(dist_right_b);
}

Node * Skeleton::get_adjusted_shoulder (unsigned int buffer_width,
		unsigned int buffer_height,
		unsigned int dimension_reduction,
		std::vector<Node *> *graph,
		Node *centroid,
		Node *head,
		Node *shoulder)
{
	Node *virtual_shoulder, *adjusted_shoulder = NULL;
	virtual_shoulder = new Node(0,0,shoulder->getX(),shoulder->getY(),shoulder->getZ());

	unsigned int i,j;

	util.convert_mm_to_screen_coords (buffer_width,
			buffer_height,
			dimension_reduction,
			virtual_shoulder->getX(),
			virtual_shoulder->getY(),
			virtual_shoulder->getZ(),
			&i,
			&j);
	virtual_shoulder->setI(i);
	virtual_shoulder->setJ(i);

	adjusted_shoulder = util.get_closest_torso_node (graph,virtual_shoulder,head);
	delete virtual_shoulder;

	return adjusted_shoulder;
}

Node * Skeleton::get_shoulder_center(unsigned int buffer_width, unsigned int buffer_height, unsigned int dimension_reduction, std::vector<Node *> * graph, Node * left_shoulder, Node * right_shoulder, Node * head) {
	Node *virtual_shoulder, *shoulder_center = NULL;
	virtual_shoulder = new Node(0,0);
	virtual_shoulder->setX(round (left_shoulder->getX() -
			(left_shoulder->getX() - right_shoulder->getX()) / 2.f));
	virtual_shoulder->setY(round (left_shoulder->getY() -
			(left_shoulder->getY() - right_shoulder->getY()) / 2.f));
	virtual_shoulder->setZ(round (left_shoulder->getZ() -
			(left_shoulder->getZ() - right_shoulder->getZ()) / 2.f));

	unsigned int i,j;

	util.convert_mm_to_screen_coords (buffer_width,
			buffer_height,
			dimension_reduction,
			virtual_shoulder->getX(),
			virtual_shoulder->getY(),
			virtual_shoulder->getZ(),
			(unsigned int *) &i,
			(unsigned int *) &j);
	virtual_shoulder->setI(i);
	virtual_shoulder->setJ(j);
	shoulder_center = util.get_closest_torso_node (graph, virtual_shoulder, head);

	delete virtual_shoulder;

	return shoulder_center;
}

std::vector<Joint *> * Skeleton::_track_joints(void) {
	Node * centroid;
	Node *head = NULL;
	Node *right_shoulder = NULL;
	Node *left_shoulder = NULL;
	Node *shoulder_center = NULL;
	Node *right_hip = NULL;
	Node *left_hip = NULL;
	std::vector<Node *> * extremas = NULL;
	std::vector<Joint *> * joints = NULL;
	std::vector<Joint *> * smoothed = NULL;

	graph = make_graph(labels);
	centroid = get_centroid();
	extremas = get_extremas(centroid);

	if (extremas != NULL && extremas->size() > 2)
	{
		if (previous_head)
		{
			int distance;
			bool can_be_head = false;
			head = util.get_closest_node_to_joint (*extremas, previous_head, &distance);
			if (head != NULL && distance < GRAPH_DISTANCE_THRESHOLD)
			{
				can_be_head = check_if_node_can_be_head (head, centroid, &left_shoulder, &right_shoulder);
			}

			if (!can_be_head)
				head = NULL;
		}

		if (head == NULL)
		{
			get_head_and_shoulders (extremas, centroid, &head, &left_shoulder, &right_shoulder);
		}

		if (joints == NULL)
			joints = new std::vector<Joint *>(SKELTRACK_JOINT_MAX_JOINTS);

		util.set_joint_from_node (joints, head, SKELTRACK_JOINT_ID_HEAD, dimension_reduction);

		if (left_shoulder && head && head->getZ() > left_shoulder->getZ())
		{
			Node *adjusted_shoulder = get_adjusted_shoulder (buffer_width,
					buffer_height,
					dimension_reduction,
					graph,
					centroid,
					head,
					left_shoulder);

			if (adjusted_shoulder)
				left_shoulder = adjusted_shoulder;
		}

		util.set_joint_from_node (joints, left_shoulder, SKELTRACK_JOINT_ID_LEFT_SHOULDER, dimension_reduction);

		if (right_shoulder && head && head->getZ() > right_shoulder->getZ())
		{
			Node *adjusted_shoulder;
			adjusted_shoulder = get_adjusted_shoulder (buffer_width,
					buffer_height,
					dimension_reduction,
					graph,
					centroid,
					head,
					right_shoulder);

			if (adjusted_shoulder)
				right_shoulder = adjusted_shoulder;
		}
		util.set_joint_from_node (joints, right_shoulder, SKELTRACK_JOINT_ID_RIGHT_SHOULDER, dimension_reduction);

		set_left_and_right_from_extremas (extremas,	head, left_shoulder, right_shoulder, joints);

		if (left_shoulder && right_shoulder)
		{
			shoulder_center = get_shoulder_center (buffer_width,buffer_height,dimension_reduction,graph,left_shoulder,right_shoulder,head);

			util.set_joint_from_node (joints,shoulder_center,SKELTRACK_JOINT_ID_SHOULDER_CENTER,dimension_reduction);
		}

		if (head && shoulder_center)
		{
			Node *virtual_center = new Node(0,0);
			unsigned int ci,cj;
			util.convert_mm_to_screen_coords (buffer_width,buffer_height,dimension_reduction,
					centroid->getX(),
					centroid->getY(),
					centroid->getZ(),
					&ci,
					&cj);
			centroid->setI(ci);
			centroid->setJ(ci);

			float angle = util.get_angle_between_nodes (shoulder_center, centroid);
			float dist = (float) util.get_distance (shoulder_center, head);
			virtual_center->setX(shoulder_center->getX() + sin(angle) * chest_factor * dist);
			virtual_center->setY(shoulder_center->getY() + cos(angle) * chest_factor * dist);
			virtual_center->setZ(shoulder_center->getZ());
			ci = 0;
			cj = 0;
			util.convert_mm_to_screen_coords (buffer_width, buffer_height, dimension_reduction,
					virtual_center->getX(),
					virtual_center->getY(),
					virtual_center->getZ(),
					&ci,
					&cj);
			virtual_center->setI(ci);
			virtual_center->setJ(cj);

			right_hip = get_shoulder_node (angle,shoulders_search_step,virtual_center->getX(),virtual_center->getY(),virtual_center->getZ());

			left_hip = get_shoulder_node (angle,-shoulders_search_step,virtual_center->getX(),virtual_center->getY(),virtual_center->getZ());

			Node *center = NULL;
			if (left_hip && right_hip)
			{
				center = new Node(0,0);

				center->setX(right_hip->getX() + (left_hip->getX() - right_hip->getX())/2.f);
				center->setY(virtual_center->getY());
				center->setZ(virtual_center->getZ());
				unsigned int cci, ccj;
				util.convert_mm_to_screen_coords (buffer_width,
						buffer_height,
						dimension_reduction,
						center->getX(),
						center->getY(),
						center->getZ(),
						&cci,
						&ccj);
			}
			else
			{
				center = virtual_center;
			}

			util.set_joint_from_node (joints, virtual_center,SKELTRACK_JOINT_ID_CENTER, dimension_reduction);

			util.set_joint_from_node (joints, right_hip,SKELTRACK_JOINT_ID_RIGHT_HIP, dimension_reduction);

			util.set_joint_from_node (joints, left_hip,SKELTRACK_JOINT_ID_LEFT_HIP, dimension_reduction);
		}
	}

	buffer = NULL;

	main_component = NULL;

	util.clean_nodes (graph);
	delete graph;
	// graph = NULL;

	util.clean_labels (labels);

	delete labels;

	if (enable_smoothing)
	{
		smooth.smooth_joints(&smooth_data, joints);

		if (smooth_data.getSmoothedJoints() != NULL)
		{
			unsigned int i;
			smoothed = new std::vector<Joint *>(SKELTRACK_JOINT_MAX_JOINTS);
			for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
			{
				Joint *smoothed_joint, *smooth, *trend;
				smoothed_joint = NULL;
				smooth = smooth_data.getSmoothedJoint(i);
				if (smooth != NULL)
				{
					if (smooth_data.getTrendJoints() != NULL)
					{
						trend = smooth_data.getTrendJoint(i);
						if (trend != NULL)
						{
							smoothed_joint = new Joint((JointId)i);
							smoothed_joint->setX(smooth->getX() + trend->getX());
							smoothed_joint->setY(smooth->getY() + trend->getY());
							smoothed_joint->setZ(smooth->getZ() + trend->getZ());
							smoothed_joint->setScreenX(smooth->getScreenX() + trend->getScreenX());
							smoothed_joint->setScreenY(smooth->getScreenY() + trend->getScreenY());
						}
						else
							smoothed_joint = smooth->copy();
					}
					else
						smoothed_joint = smooth->copy();
				}
				smoothed->at(i) = smoothed_joint;
			}
		}
		delete joints;

		joints = smoothed;
	}

	if (joints)
	{
		Joint *joint = joints->at((int)SKELTRACK_JOINT_ID_HEAD);
		if (joint != NULL)
		{
			delete previous_head;
			previous_head = joint->copy();
		}
	}

	delete extremas; // created elsewhere

	return joints;
}

void Skeleton::clean_tracking_resources(void) {
	free(distances_matrix);
	distances_matrix = NULL;

	free(node_matrix);
	node_matrix = NULL;
}

void Skeleton::track_joints_in_thread(void)
{
	std::vector<Joint *> * joints = _track_joints();

	track_joints_mutex->lock();
	track_joints_result = NULL;
	track_joints_result = joints;
	track_joints_mutex->unlock();
}

/**
 * skeltrack_skeleton_set_focus_point
 * @param x: The x coordinate of the focus point.
 * @param y: The y coordinate of the focus point.
 * @param z: The z coordinate of the focus point.
 *
 * Gets the focus point which is the origin from where the tracking will
 * start. The coordinates will be in mm.
 *
 **/
void Skeleton::get_focus_point(int *x, int *y, int *z)
{
	*x = focus_node->getX();
	*y = focus_node->getY();
	*z = focus_node->getZ();
}

/**
 * skeltrack_skeleton_set_focus_point
 * @param x: The x coordinate of the focus point.
 * @param y: The y coordinate of the focus point.
 * @param z: The z coordinate of the focus point.
 *
 * Sets the focus point which is the origin from where the tracking will
 * start. The coordinates should be in mm.
 *
 * If this method is not called the default values are @x = 0, @y = 0,
 * @z = 1000, that is, in the center of the screen and at 1m from the
 * camera.
 *
 **/
void Skeleton::set_focus_point (int x, int y, int z)
{
	focus_node->setPosition(x,y,z);
}

/**
 * skeltrack_skeleton_track_joints:
 * @buffer: The buffer containing the depth information, from which
 * all the information will be retrieved.
 * @width: The width of the @buffer
 * @height: The height of the @buffer
 * @callback: (scope async): The function to call when the it is finished
 * tracking the joints
 *
 * Tracks the skeleton's joints.
 *
 * It uses the depth information contained in the given @buffer and tries to
 * track the skeleton joints. The @buffer's depth values should be in mm.
 * Use skeltrack_skeleton_track_joints_finish() to get a list of the joints
 * found.
 *
 * If this method is called while a previous attempt of tracking the joints
 * is still running, a %G_IO_ERROR_PENDING error occurs.
 *
 **/
void Skeleton::track_joints(unsigned short *buffer, unsigned int width, unsigned int height)
{
	track_joints_mutex->lock();

	this->buffer = buffer;

	track_joints_mutex->unlock();

	if (buffer_width != width || buffer_height != height)
	{
		clean_tracking_resources();

		buffer_width = width;
		buffer_height = height;
	}

	track_joints_in_thread();
}

/**
 * skeltrack_skeleton_track_joints_finish:
 * @self: The #SkeltrackSkeleton
 * @result: The #GAsyncResult provided in the callback
 * @error: (allow-none): A pointer to a #GError, or %NULL
 *
 * Gets the list of joints that were retrieved by a
 * skeltrack_skeleton_track_joints() operation.
 *
 * Use skeltrack_joint_list_get_joint() with #SkeltrackJointId
 * to get the respective joints from the list.
 * Joints that could not be found will appear as %NULL in the list.
 *
 * The list should be freed using skeltrack_joint_list_free().
 *
 * Returns: (transfer full): The #SkeltrackJointList with the joints found.
 */
std::vector<Joint *> * Skeleton::track_joints_finish(void)
{
	return track_joints_result;
}

/**
 * skeltrack_skeleton_track_joints_sync:
 * @buffer: The buffer containing the depth information, from which
 * all the information will be retrieved.
 * @width: The width of the @buffer
 * @height: The height of the @buffer
 *
 * Tracks the skeleton's joints synchronously.
 *
 * Does the same as skeltrack_skeleton_track_joints() but synchronously
 * and returns the list of joints found.
 * Ideal for off-line skeleton tracking.
 *
 * If this method is called while a previous attempt of asynchronously
 * tracking the joints is still running, a %G_IO_ERROR_PENDING error occurs.
 * *
 * Returns: (transfer full): The #SkeltrackJointList with the joints found.
 **/
std::vector<Joint *> * Skeleton::track_joints_sync(unsigned short *buffer, unsigned int width, unsigned int height)
{
	this->buffer = buffer;

	if (buffer_width != width || buffer_height != height)
	{
		clean_tracking_resources();

		buffer_width = width;
		buffer_height = height;
	}

	return _track_joints();
}

} /* namespace Skeltrack */
