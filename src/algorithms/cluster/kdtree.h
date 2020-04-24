#ifndef KDTREE_H_
#define KDTREE_H_

// #include "../../render/render.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

std::vector<int> medianSortPoints(const std::vector<std::vector<float>> &points);

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
	int treeDepth;

	KdTree()
	: root(NULL), treeDepth(0)
	{}

	void insert(const std::vector<float> &point, int id)
	{
		// Insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		
		// Start with root node, at depth 0
		// Recursive function
		insertHelper(root, 0, point, id);
	}

	void insertHelper(Node *&node, int depth, const std::vector<float> &point, int id)
	{
		if (node == NULL) 
		{
			node = (new Node(point, id));
			if (depth > treeDepth)
			{
				treeDepth = depth;
			}
		} 
		else
		{
			// Calculate dimension to split on, at this depth of the KD-Tree
			// Example:
			// depth = 0: split on 1st dimension (e.g. x)
			// depth = 1: split on 2nd dimension (e.g. y)
			// depth = 2: split on 3rd dimension (e.g. z)
			int dim = depth % 3; // 3-dimensions

			if (point[dim] < node->point[dim])
			{
				insertHelper(node->left, depth+1, point, id);
			} 
			else 
			{
				insertHelper(node->right, depth+1, point, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// Recursive function
		searchHelper(ids, root, 0, target, distanceTol);
		return ids;
	}
	
	void searchHelper(std::vector<int> &ids, Node *&node, int depth, const std::vector<float> &target, float distanceTol)
	{
		// Exit if we're at a leaf node
		if (node == NULL)
		{
			return;
		}

		// Check if current node is within 1-norm (e.g. box) of target
		float dx = target[0] - node->point[0];
		if (dx < distanceTol || dx > -distanceTol){
			float dy = target[1] - node->point[1];
			if (dy < distanceTol || dy > -distanceTol){
				float dz = target[2] - node->point[2];
				if (dz < distanceTol || dz > -distanceTol){

					// Check if node is within 2-norm (e.g. circle) of target
					if (dx*dx + dy*dy + dz*dz <= distanceTol*distanceTol)
					{
						// Node is within 2-norm, so add to ids
						ids.push_back(node->id);
					}
				}
			}
		}

		// Check if we should search further
		// Calculate dimension to check, at this depth of the KD-Tree
		int dim = depth % 3; // 3-dimensions
		float ddim = target[dim] - node->point[dim];
		
		if (ddim < distanceTol)
		{
			// Search left side of tree
			searchHelper(ids, node->left, depth+1, target, distanceTol);
		}
		if (ddim > -distanceTol)
		{
			// Search right side of tree
			searchHelper(ids, node->right, depth+1, target, distanceTol);
		}
	}

};

#endif /* CLUSTER_H_ */