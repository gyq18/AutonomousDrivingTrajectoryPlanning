#pragma once

struct MyNode {
	double x, y, cost;
	int parent;
	MyNode(double x_, double y_) : x(x_), y(y_), cost(0), parent(-1) {}
	MyNode() : x(0), y(0), cost(0), parent(-1) {}
};
