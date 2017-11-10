#include "Graph.h"

Connection::Connection() {

}

Connection::Connection(Vector2D _from, Vector2D _to, float _cost) {
	this->from = _from;
	this->to = _to;
	this->cost = _cost;
}

Connection::~Connection() {

}

float Connection::getCost() {
	return this->cost;
}

Vector2D* Connection::getFromNode() {
	return &this->from;
}

Vector2D* Connection::getToNode() {
	return &this->to;
}

Graph::Graph() {
	
}

Graph::~Graph() {

}

vector<Connection> Graph::getConnections(Vector2D* fromNode) {
	vector<Connection> con;
	for (int i = 0; i < this->connections.size(); i++) {
		if (*fromNode == *(this->connections[i].getFromNode())) 
			con.push_back(this->connections[i]);
	}
	return con;
}