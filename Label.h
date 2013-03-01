/*
 * Lable.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_LABEL_H_
#define SKELTRACK_LABEL_H_

#include <cstddef>
#include <vector>
#include "Node.h"

namespace Skeltrack {

class Label {
public:
	Label(int index) {
		this->index = index;
		this->parent = this;
		this->bridge_node = NULL;
		this->to_node = NULL;
		lower_screen_y = -1;
		lower_z = -1;
		higher_z = -1;
		normalized_num_nodes = -1.0;
		nodes = NULL;
	}
	virtual ~Label() {
		if (nodes != NULL) {
			nodes->clear();
			delete nodes;
		}
	}

	int getIndex(void) { return index; }

	void setParent(Label * parent) { this->parent = parent; }
	Label * getParent(void) { return parent; }

	std::vector<Node *> * getNodes() { return nodes; }
	void appendNode(Node * node) {
		if (nodes == NULL) nodes = new std::vector<Node *>();
		nodes->push_back(node);
	}

	Node * getBridgeNode(void) { return bridge_node; }
	void setBridgeNode(Node * node) { bridge_node = node; }

	Node * getToNode(void) { return to_node; }
	void setToNode(Node * node) { to_node = node; }

	void setLowerScreenY(int y) { lower_screen_y = y; }
	int getLowerScreenY(void) { return lower_screen_y; }

	void setHigherZ(int z) { higher_z = z; }
	int getHigherZ(void) { return higher_z; }

	void setLowerZ(int z) { lower_z = z; }
	int getLowerZ(void) { return lower_z; }

	double getNormalizedNumNodes(void) { return normalized_num_nodes; }
	void setNormalizedNumNodes(double value) { normalized_num_nodes = value; }
protected:
	  int index;
	  Label * parent;
	  std::vector<Node *> * nodes;
	  Node * bridge_node;
	  Node * to_node;
	  int lower_screen_y;
	  int higher_z;
	  int lower_z;
	  double normalized_num_nodes;
};

} /* namespace Skeltrack */
#endif /* SKELTRACK_LABLE_H_ */
