/*
 * PQueue.cpp
 *
 *  Created on: 23 de Fev de 2013
 *      Author: fahrenheit
 */

#include "PQueue.h"

namespace Skeltrack {

PQueue::PQueue(unsigned int max_size, unsigned int width, unsigned int height) {
	unsigned int i;

	elements = new std::vector<PQelement * >(max_size+1);
	map = new std::vector<int>(width * height);

	for (i=0; i<width*height; i++)
		map->at(i) = -1;

	this->_size = 0;
	this->max_size = max_size;
	this->width = width;
	this->height = height;
}

PQueue::~PQueue() {
	delete elements;
	delete map;
}

unsigned int PQueue::indexOf(Node * element) {
	std::vector<PQelement * >::iterator it;
	unsigned int i = 0;
	for (it = elements->begin(); it != elements->end(); it++) {
		if ((*it)->data == element) return i;
		i++;
	}
	return _size+1;
}

void PQueue::swap (unsigned int a, unsigned int b)
{
	unsigned int index_a, index_b;

	index_a = elements->at(a)->data->getJ() * width + elements->at(a)->data->getI();
	index_b = elements->at(b)->data->getJ() * width + elements->at(b)->data->getI();

	unsigned int temp = map->at(index_a);
	map->at(index_a) = map->at(index_b);
	map->at(index_b) = temp;

	PQelement * element_temp = elements->at(b);
	elements->at(b) = elements->at(a);
	elements->at(a) = element_temp;
}

bool PQueue::greater (unsigned int a, unsigned int b)
{
	return elements->at(a)->priority > elements->at(b)->priority;
}

void PQueue::swim (unsigned int index)
{
	while (index > 1 && greater (index/2, index))
	{
		swap(index, index/2);
		index = index/2;
	}
}

void PQueue::sink (unsigned int index)
{
	unsigned int j;

	while (2 * index <= _size)
	{
		j = 2 * index;

		if ((j < _size) && (greater(j, j+1)))
			j++;

		if (!greater (index, j))
			break;

		swap (index, j);

		index = j;
	}
}

void PQueue::insert(Node * node, int priority) {
	unsigned int index = 0;

	unsigned int idx = ++(_size);

	if (elements->at(idx) == NULL)
		elements->at(idx) = new PQelement(node,priority);
	else {
		elements->at(idx)->data = node;
		elements->at(idx)->priority = priority;
	}

	index = node->getJ() * width + node->getI();
	map->at(index) = _size;

	swim(_size);
}

Node * PQueue::pop_minimum(void) {
	if (empty())
		return NULL;

	Node *minimum = elements->at(1)->data;
	unsigned int index;
	swap(1, _size);
	_size--;

	index = minimum->getJ() * width + minimum->getI();
	map->at(index) = -1;

	sink(1);
	return minimum;
}

void PQueue::remove(Node *data)
{
	unsigned int index, pos;

	index = data->getJ() * width + data->getI();
	pos = map->at(index);

	Node *element = elements->at(pos)->data;

	swap(pos, _size);
	_size--;

	map->at(element->getJ() * width + element->getI()) = -1;

	sink (pos);
}

bool PQueue::has_element(Node * data) {
	return map->at(data->getJ() * width + data->getI()) != -1;
}



} /* namespace CSIVega */
