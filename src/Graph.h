#pragma once
#include <vector>
#include "Vector2D.h"

using namespace std;

class Connection {
public:
	Connection();
	Connection(Vector2D _from, Vector2D _to, float _cost);
	~Connection();

	Vector2D from;
	Vector2D to;
	float cost;

	float getCost();
	Vector2D* getFromNode();
	Vector2D* getToNode();

};

class Graph {
public:
	Graph();
	~Graph();

	vector<Connection> connections;

	vector<Connection> getConnections(Vector2D* fromNode);

};