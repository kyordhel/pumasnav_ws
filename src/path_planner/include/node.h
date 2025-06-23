#pragma once
#include <memory>

namespace path_planner{

class Node;
typedef std::shared_ptr<Node> NodePtr;


class Node{
public:
	/**
	 * The index of the corresponding cell in the occupancy grid.
	 */
	int   index;

	/**
	 * The accumulated distance of this node.
	 */
	float g_value;

	/**
	 * The f-value, used only in the A* algorithm.
	 */
	float f_value;

	/**
	 * A value indicating whether this node is in the open list or not.
	 */
	bool  in_open_list;

	/**
	 * A value indicating whether this node is in the closed list or not.
	 */
	bool  in_closed_list;

	/**
	 * A pointer to the parent of this node.
	 */
	NodePtr parent;

private:
	Node(int index);
	Node(const Node&) = delete;
	Node& operator=(const Node&) = delete;

public:
	static NodePtr makeShared(int index);

public:
	/**
	 * Compare type for F value required by priority_queue.
	 * Returns true if the f_value n1 is greater than that of n2
	 */
	struct CompareByFValue{
		bool operator()(const NodePtr n1, const NodePtr n2);
	};
	/**
	 * Compare type for G value required by priority_queue.
	 * Returns true if the g_value n1 is greater than that of n2
	 */
	struct CompareByGValue{
		bool operator()(const NodePtr n1, const NodePtr n2);
	};
};

}