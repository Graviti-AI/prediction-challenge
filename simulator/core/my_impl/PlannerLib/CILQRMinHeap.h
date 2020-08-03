#ifndef CILQR_MIN_HEAP_H
#define CILQR_MIN_HEAP_H

#include <vector>
#include "CILQRTypeDef.h"

class CMinHeap
{
private:
	void bubbleDown(const int index);
	void bubbleUp(const int index);
	void heapify();
	std::vector<CNode> heap;

public:
	CMinHeap(){};
	CMinHeap(const CNode& node);

	void insert(const CNode& newNode);
	CNode getMin();
	void deleteMin();
	int getSize();
	void clearHeap();
};

#endif