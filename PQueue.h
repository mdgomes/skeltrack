/*
 * PQueue.h
 *
 *  Created on: 23 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_PQUEUE_H_
#define SKELTRACK_PQUEUE_H_

#include <cstdlib>
#include <cmath>
#include <vector>

#include "Util.h"
#include "Node.h"
#include "PQelement.h"

namespace Skeltrack {

class PQueue {
public:
	PQueue(unsigned int max_size, unsigned int width, unsigned int height);
	virtual ~PQueue();

	virtual void insert(Node * data, int priority);
	virtual Node * pop_minimum(void);
	virtual unsigned int size(void) { return _size; }
	virtual Node * top(void) { return pop_minimum(); }
	virtual void remove(Node * node);
	virtual bool has_element(Node * element);
	virtual bool empty(void) { return _size == 0; };
protected:
	std::vector<int> *map;
	std::vector<PQelement *> * elements;
	unsigned int _size;
	unsigned int max_size;
	unsigned int width;
	unsigned int height;

	unsigned int indexOf(Node * element);

	void swap (unsigned int a, unsigned int b);
	bool greater (unsigned int a, unsigned int b);
	void swim (unsigned int index);
	void sink (unsigned int index);
};

} /* namespace Skeltrack */
#endif /* SKELTRACK_PQUEUE_H_ */
