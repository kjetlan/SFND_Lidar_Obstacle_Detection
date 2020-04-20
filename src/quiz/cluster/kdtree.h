/* \author Aaron Brown */
// Quiz on implementing kd tree

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
			// depth = 0: split on x-dimension (dim = 0)
			// depth = 1: split on y-dimension (dim = 1)
			// depth = 2: split on x-dimension (dim = 0)
			// depth = 4: split on y-dimension (dim = 1)
			uint dim = depth % 2;

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

		// Check if current node is within 1-norm of target
		float dx = abs(node->point[0] - target[0]);
		float dy = abs(node->point[1] - target[1]);
		if (dx <= distanceTol && dy <= distanceTol)
		{
			// Check if node is within 2-norm of target
			float dist = sqrt(dx*dx + dy*dy);

			if (dist <= distanceTol)
			{
				// Node is within 2-norm, so add to ids
				ids.push_back(node->id);
			}
		}

		// Check if we should search further
		uint dim = depth % 2;

		// Search left side of tree
		// if targets lowest x/y value is lower than current nodes x/y value
		if (target[dim] - distanceTol < node->point[dim])
		{
			searchHelper(ids, node->left, depth+1, target, distanceTol);
		}
		// Search right side of tree
		// if targets highest x/y value is higher than current nodes x/y value
		if (target[dim] + distanceTol > node->point[dim])
		{
			searchHelper(ids, node->right, depth+1, target, distanceTol);
		}
	}

};




