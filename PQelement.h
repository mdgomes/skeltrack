/*
 * PQelement.h
 *
 *  Created on: 23 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_PQELEMENT_H_
#define SKELTRACK_PQELEMENT_H_

#include "Node.h"

namespace Skeltrack {

class PQelement {
public:
	Node * data;
	unsigned int priority;

	PQelement(Node * data, unsigned int priority) {
		this->data = data;
		this->priority = priority;
	}

	virtual ~PQelement(void) {};

	friend class PQueue;
};

} /* namespace CSIVega */
#endif /* PQELEMENT_H_ */
