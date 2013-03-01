/*
 * Node.h
 *
 *  Created on: 11 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_NODE_H_
#define SKELTRACK_NODE_H_

#include <list>
#include <cstdlib>
#include <cmath>
// #include "Label.h"

namespace Skeltrack {

class Label;

class Node {
public:
	Node(int i, int j, int x = 0, int y = 0, int z = 0) {
		this->i = i;
		this->j = i;
		this->x = x;
		this->y = y;
		this->z = z;
		label = NULL;
		this->neighbors = NULL;
		this->linked_nodes = NULL;
	}
	virtual ~Node() {
		// delete structures
		if (neighbors != NULL) {
			neighbors->clear();
			delete neighbors;
		}
		if (linked_nodes != NULL) {
			linked_nodes->clear();
			delete linked_nodes;
		}
	}

	int getX(void) { return x; }
	void setX(int value) { x = value; }
	int getY(void) { return y; }
	void setY(int value) { y = value; }
	int getZ(void) { return z; }
	void setZ(int value) { z = value; }

	void setPosition(int x, int y, int z) { this->x = x; this->y = y; this->z = z; }

	void setScreenPosition(int i, int j) { this->i = i; this->j = j; }

	int getI(void) { return i; }
	void setI(int value) { i = value; }
	int getJ(void) { return j; }
	void setJ(int value) { j = value; }

	void setLabel(Label * label) { this->label = label; }
	Label * getLabel(void) { return label; }

	std::list<Node *> * getNeighbours(void) { return neighbors; }
	std::list<Node *> * getLinkedNodes(void) { return linked_nodes; }

	void appendNeighbour(Node * node) {
		if (neighbors == NULL) neighbors = new std::list<Node *>();
		neighbors->push_back(node);
	}

	void appendLinkedNode(Node * node) {
		if (linked_nodes == NULL) linked_nodes = new std::list<Node *>();
		linked_nodes->push_back(node);
	}

	void removeNodeFromNeighbours(Node * node) {
		if (node == NULL || neighbors == NULL) return;
		remove(neighbors,node);
	}

	void removeNodeFromLinkedNodes(Node * node) {
		if (node == NULL || linked_nodes == NULL) return;
		remove(linked_nodes,node);
	}

	Node * neighbors_at(unsigned int pos) { return at(neighbors,pos); }

	Node * linked_nodes_at(unsigned int pos) { return at(linked_nodes,pos); }

	/**
	 * Gets the distance to another node.
	 * @param node Node to which we want the distance
	 * @return distance
	 */
	double distanceToNode(Node * node) {
		double dx, dy, dz;
		dx = (double)abs(node->getX() - this->x);
		dy = (double)abs(node->getY() - this->y);
		dz = (double)abs(node->getZ() - this->z);
		return sqrt(dx*dx + dy*dy + dz*dz);
	}
protected:
	int i;
	int j;
	int x;
	int y;
	int z;
	std::list<Node *> * neighbors;
	std::list<Node *> * linked_nodes;
	Label * label;

	std::list<Node *>::iterator find(std::list<Node *> * vec, Node * target) {
		std::list<Node *>::iterator it;
		for (it = vec->begin(); it != vec->end(); it++) {
			if ((*it) == target) return it;
		}
		return vec->end();
	}
	std::list<Node *>::iterator remove(std::list<Node *> * vec, Node * target) {
		std::list<Node *>::iterator it;
		it = find(vec,target);
		if (it != vec->end())
			return vec->erase(it);
		else
			return vec->end();
	}

	Node * at(std::list<Node *> * vec, unsigned int pos) {
		if (vec == NULL || pos > vec->size()) return NULL; // at least doesn't
		unsigned int i = 0;
		for (std::list<Node *>::iterator it = vec->begin(); it != vec->end(); it++) {
			if (i == pos)
				return (*it);
			i++;
		}
		return NULL; // couldn't find the node
	}
};

} /* namespace Skeltrack */
#endif /* NODE_H_ */
