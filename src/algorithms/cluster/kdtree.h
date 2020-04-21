#ifndef KDTREE_H_
#define KDTREE_H_

#include "../../render/render.h"


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

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		
		// Start with root node, at depth 0
		// Recursive function
		insertHelper(root, 0, point, id);
	}

	void insertHelper(Node *&node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL) 
		{
			node = (new Node(point, id));
		} 
		else
		{
			// Calculate dimension to split on, at this depth of the KD-Tree
			// Example:
			// depth = 0: split on 1st dimension (e.g. x)
			// depth = 1: split on 2nd dimension (e.g. y)
			// depth = 2: split on 3rd dimension (e.g. z)
			uint dim = depth % point.size();

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
	
	void searchHelper(std::vector<int> &ids, Node *&node, uint depth, std::vector<float> target, float distanceTol)
	{
		// Exit if we're at a leaf node
		if (node == NULL)
		{
			return;
		}

		// Check if current node is within 1-norm (e.g. box) of target
		bool isInsideBox = false;
		std::vector<float> deltas(target.size(),0.0);

		for (int i = 0; i < target.size(); i++)
		{	
			deltas[i] = abs(node->point[i] - target[i]);
			
			if (deltas[i] <= distanceTol)
			{
				isInsideBox = true;
			}
			else
			{
				isInsideBox = false;
				break;
			}
		}
		
		if (isInsideBox)
		{
			// Check if node is within 2-norm (e.g. circle) of target
			float dist = 0.0;
			for (float delta : deltas)
			{
				dist += delta*delta;
			}
			dist = sqrt(dist);

			if (dist <= distanceTol)
			{
				// Node is within 2-norm, so add to ids
				ids.push_back(node->id);
			}
		}

		// Check if we should search further

		// Calculate dimension to check, at this depth of the KD-Tree
		uint dim = depth % target.size();

		// Search left side of tree
		// if targets lowest x/y/z.. value is lower than current nodes x/y/z.. value
		if (target[dim] - distanceTol < node->point[dim])
		{
			searchHelper(ids, node->left, depth+1, target, distanceTol);
		}
		// Search right side of tree
		// if targets highest x/y/z.. value is higher than current nodes x/y/z.. value
		if (target[dim] + distanceTol > node->point[dim])
		{
			searchHelper(ids, node->right, depth+1, target, distanceTol);
		}
	}

};

#endif /* CLUSTER_H_ */