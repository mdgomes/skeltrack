/*
 * Test.h
 *
 *  Created on: 13 de Fev de 2013
 *      Author: fahrenheit
 */

#ifndef SKELTRACK_TEST_H_
#define SKELTRACK_TEST_H_

#include "Skeltrack.h"
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>

namespace Skeltrack {

#define WIDTH  640
#define HEIGHT 480

#define NUMBER_OF_FILES 12
#define RESOURCES_FOLDER "./resources/"
static char *DEPTH_FILES[NUMBER_OF_FILES] = {
		RESOURCES_FOLDER "depth-data-1028894671",
		RESOURCES_FOLDER "depth-data-1045879925",
		RESOURCES_FOLDER "depth-data-1058893191",
		RESOURCES_FOLDER "depth-data-1070905432",
		RESOURCES_FOLDER "depth-data-1166565565",
		RESOURCES_FOLDER "depth-data-1038901490",
		RESOURCES_FOLDER "depth-data-1051883281",
		RESOURCES_FOLDER "depth-data-1064898470",
		RESOURCES_FOLDER "depth-data-1078881076",
		RESOURCES_FOLDER "depth-data-1234568668",
		RESOURCES_FOLDER "depth-data-1399145206",
		RESOURCES_FOLDER "depth-data-82823944"};

typedef struct
{
	unsigned short *data;
	unsigned int width;
	unsigned int height;
} Buffer;

typedef struct
{
	Skeleton *skeleton;
	Buffer *buffer;
} Fixture;

class Test {
public:
	Test();
	virtual ~Test();

	void loop(void);
protected:
	unsigned short * read_file_to_buffer(const char * name, unsigned int count) {
		unsigned short * depth = NULL;
		std::ifstream infile;
		infile.open(name,std::ios::in|std::ios::binary|std::ios::ate);
		std::ifstream::pos_type size;
		if (infile) {
			size = infile.tellg();
			char * memblock = new char[size];
			infile.seekg(0, std::ios::beg);
			infile.read(memblock, size);
			infile.close();
			depth = (unsigned short *)memblock;
		}
		return depth;
	}

	unsigned short * reduce_depth_file(const char *name, unsigned int reduce_factor, unsigned int *reduced_width, unsigned int *reduced_height)
	{
		unsigned int i, j, r_width, r_height;
		unsigned short *depth, *reduced_depth;
		unsigned int count = WIDTH * HEIGHT * sizeof (unsigned short);

		depth = read_file_to_buffer(name, count);

		if (depth == NULL)
			return NULL;

		r_width = (WIDTH - WIDTH % reduce_factor) / reduce_factor;
		r_height = (HEIGHT - HEIGHT % reduce_factor) / reduce_factor;
		reduced_depth = (unsigned short*) malloc(r_width * r_height * sizeof (unsigned short));

		for (i = 0; i < r_width; i++)
		{
			for (j = 0; j < r_height; j++)
			{
				unsigned int index = j * WIDTH * reduce_factor + i * reduce_factor;
				reduced_depth[j * r_width + i] = depth[index];
			}
		}
		*reduced_width = r_width;
		*reduced_height = r_height;

		delete depth;
		return reduced_depth;
	}

	int get_number_of_valid_joints (std::vector<Joint *> list)
	{
		int i;
		int count = 0;

		for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
		{
			if (list[i] != NULL)
				count++;
		}
		return count;
	}

	void test_track_joints_number (Fixture *f, void * test_data)
	{
		char *file_name = NULL;
		Buffer *buffer = NULL;
		unsigned int reduction, width, height;
		unsigned short *depth = NULL;

		file_name = (char *) test_data;

		reduction = (unsigned int) f->skeleton->getProperty(Skeltrack::PROP_DIMENSION_REDUCTION);

		depth = reduce_depth_file (file_name, reduction, &width, &height);
		buffer = new Buffer();
		buffer->data = depth;
		buffer->width = width;
		buffer->height = height;
		f->buffer = buffer;
		f->skeleton->track_joints(depth, width, height);
	}
};

} /* namespace Skeltrack */
#endif /* TEST_H_ */
