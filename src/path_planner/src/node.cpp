#include "node.h"
#include <climits>
using namespace path_planner;

Node::Node(int index):
	index(index), g_value(INT_MAX), f_value(INT_MAX), in_open_list(false),
	in_closed_list(false), parent(nullptr){}

NodePtr Node::makeShared(int index){
	return std::shared_ptr<Node>(new Node(index));
}

bool Node::CompareByGValue::operator()(NodePtr n1, NodePtr n2) {
	if((n1 != nullptr) && (n1 != nullptr))
		return n1->g_value > n2->g_value;
	return false;
}

bool Node::CompareByFValue::operator()(NodePtr n1, NodePtr n2) {
	if((n1 != nullptr) && (n1 != nullptr))
		return n1->f_value > n2->f_value;
	return false;
}
