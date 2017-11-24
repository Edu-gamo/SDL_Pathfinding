#include "Agent.h"
#include <queue>
#include <cmath>

using namespace std;

Agent::Agent() : sprite_texture(0),
                 position(Vector2D(100, 100)),
	             target(Vector2D(1000, 100)),
	             velocity(Vector2D(0,0)),
	             mass(0.1f),
	             max_force(150),
	             max_velocity(200),
	             orientation(0),
	             color({ 255,255,255,255 }),
				 sprite_num_frames(0),
	             sprite_w(0),
	             sprite_h(0),
	             draw_sprite(false)
{
	steering_behavior = new SteeringBehavior;
}

Agent::~Agent()
{
	if (sprite_texture)
		SDL_DestroyTexture(sprite_texture);
	if (steering_behavior)
		delete (steering_behavior);
}

SteeringBehavior * Agent::Behavior()
{
	return steering_behavior;
}

Vector2D Agent::getPosition()
{
	return position;
}

Vector2D Agent::getTarget()
{
	return target;
}

Vector2D Agent::getVelocity()
{
	return velocity;
}

float Agent::getMaxVelocity()
{
	return max_velocity;
}

void Agent::setPosition(Vector2D _position)
{
	position = _position;
}

void Agent::setTarget(Vector2D _target)
{
	target = _target;
}

void Agent::setVelocity(Vector2D _velocity)
{
	velocity = _velocity;
}

void Agent::setMass(float _mass)
{
	mass = _mass;
}

void Agent::setColor(Uint8 r, Uint8 g, Uint8 b, Uint8 a)
{
	color = { r, g, b, a };
}

void Agent::update(Vector2D steering_force, float dtime, SDL_Event *event)
{

	//cout << "agent update:" << endl;

	switch (event->type) {
		/* Keyboard & Mouse events */
	case SDL_KEYDOWN:
		if (event->key.keysym.scancode == SDL_SCANCODE_SPACE)
			draw_sprite = !draw_sprite;
		break;
	default:
		break;
	}


	Vector2D acceleration = steering_force / mass;
	velocity = velocity + acceleration * dtime;
	velocity = velocity.Truncate(max_velocity);

	position = position + velocity * dtime;


	// Update orientation
	if (velocity.Length()>0)
		orientation = (float)(atan2(velocity.y, velocity.x) * RAD2DEG);


	// Trim position values to window size
	if (position.x < 0) position.x = TheApp::Instance()->getWinSize().x;
	if (position.y < 0) position.y = TheApp::Instance()->getWinSize().y;
	if (position.x > TheApp::Instance()->getWinSize().x) position.x = 0;
	if (position.y > TheApp::Instance()->getWinSize().y) position.y = 0;
}

void Agent::draw()
{
	if (draw_sprite)
	{
		Uint32 sprite;
		
		if (velocity.Length() < 5.0)
			sprite = 1;
		else
			sprite = (int)(SDL_GetTicks() / (max_velocity)) % sprite_num_frames;
		
		SDL_Rect srcrect = { (int)sprite * sprite_w, 0, sprite_w, sprite_h };
		SDL_Rect dstrect = { (int)position.x - (sprite_w / 2), (int)position.y - (sprite_h / 2), sprite_w, sprite_h };
		SDL_Point center = { sprite_w / 2, sprite_h / 2 };
		SDL_RenderCopyEx(TheApp::Instance()->getRenderer(), sprite_texture, &srcrect, &dstrect, orientation+90, &center, SDL_FLIP_NONE);
	}
	else 
	{
		draw_circle(TheApp::Instance()->getRenderer(), (int)position.x, (int)position.y, 15, color.r, color.g, color.b, color.a);
		SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), (int)position.x, (int)position.y, (int)(position.x+15*cos(orientation*DEG2RAD)), (int)(position.y+15*sin(orientation*DEG2RAD)));
	}
}

bool Agent::loadSpriteTexture(char* filename, int _num_frames)
{
	if (_num_frames < 1) return false;

	SDL_Surface *image = IMG_Load(filename);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	sprite_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	sprite_num_frames = _num_frames;
	sprite_w = image->w / sprite_num_frames;
	sprite_h = image->h;
	draw_sprite = true;

	if (image)
		SDL_FreeSurface(image);

	return true;
}

//Pathfinding
Path Agent::pathFind(Vector2D pinit, Vector2D pend, Graph terrain) {

	switch (method) {
		case 0://Breadth First Search
			return breadthFirstSearch(pinit, pend, terrain);
			break;
		case 1://Dijkstra
			return dijkstra(pinit, pend, terrain);
			break;
		case 2://Greedy Best-First-Search
			return greedyBestFirstSearch(pinit, pend, terrain);
			break;
		case 3://A*
			return apuntero(pinit, pend, terrain);
			break;
	}

}

bool search(vector<Connection> cameFrom, Vector2D position) {
	bool find = false;
	int i = 0;
	while (!find && i < cameFrom.size()) {
		if (*cameFrom[i].getToNode() == position) 
			find = true;
		i++;
	}
	return find;
}

int find(vector<Connection> cameFrom, Vector2D position) {
	int pos;
	bool find = false;
	int i = 0;
	while (!find && i < cameFrom.size()) {
		if (*cameFrom[i].getToNode() == position) {
			find = true;
			pos = i;
		}
		i++;
	}
	return pos;
}

Vector2D getPrevious(vector<Connection> cameFrom, Vector2D position) {
	bool find = false;
	Vector2D prev = NULL;
	int i = 0;
	while (!find && i < cameFrom.size()) {
		if (*cameFrom[i].getToNode() == position) {
			find = true;
			prev = *cameFrom[i].getFromNode();
		}
		i++;
	}
	return prev;
}

Path Agent::breadthFirstSearch(Vector2D pinit, Vector2D pend, Graph terrain) {

	queue<Vector2D> frontier;
	frontier.push(pinit);
	vector<Connection> cFrom;
	cFrom.push_back(Connection(NULL, pinit, 0));

	int cont = 0; //Contador de nodos explorados

	while (frontier.size() > 0){

		Vector2D current = frontier.front();
		frontier.pop();
		//if (current == pend) break; //Early exit
		
		vector<Connection> neighboor = terrain.getConnections(&current);
		cont++;

		for (int i = 0; i < neighboor.size(); i++) {
			if (!search(cFrom, *neighboor[i].getToNode())) {
				frontier.push(*neighboor[i].getToNode());
				cFrom.push_back(neighboor[i]);
			}
		}

	}

	cout << "Nodos explorados (breadthFirstSearch): " << cont << endl;

	Path pathInverse;
	Vector2D current = pend;
	pathInverse.points.push_back(current);
	while (current != pinit && current != NULL) {

		current = getPrevious(cFrom, current);
		if(current != NULL) pathInverse.points.push_back(current);

	}

	cFrom.clear();
	
	int size = pathInverse.points.size();
	Path path;
	for (int i = 0; i < size; i++) {
		path.points.push_back(pathInverse.points.back());
		pathInverse.points.pop_back();
	}

	return path;

}

bool operator<(pair<int, Vector2D> a, pair<int, Vector2D> b) { return a.first > b.first ? true : false; }

Path Agent::dijkstra(Vector2D pinit, Vector2D pend, Graph terrain) {

	priority_queue<pair<int, Vector2D>> frontier;
	frontier.push(make_pair(0, pinit));
	vector<Connection> cFrom;
	cFrom.push_back(Connection(NULL, pinit, 0));

	int cont = 0; //Contador de nodos explorados

	while (frontier.size() > 0) {

		pair<int, Vector2D> current = frontier.top();
		frontier.pop();
		if (current.second == pend) break; //Early exit

		vector<Connection> neighboor = terrain.getConnections(&current.second);
		cont++;

		for (int i = 0; i < neighboor.size(); i++) {
			int newCost = current.first + neighboor[i].cost;
			bool isthere = search(cFrom, *neighboor[i].getToNode());
			if (!isthere) {
				neighboor[i].cost = newCost;
				frontier.push(make_pair(newCost, *neighboor[i].getToNode()));
				cFrom.push_back(neighboor[i]);
			} else {
				int pos = find(cFrom, *neighboor[i].getToNode());
				int provaCost = cFrom[pos].cost;
				if (provaCost < 0) cout << "ERROR" << endl;
				if (newCost < provaCost) {
					neighboor[i].cost = newCost;
					frontier.push(make_pair(newCost, *neighboor[i].getToNode()));
					cFrom[pos] = neighboor[i];
				}
			}
		}

	}

	cout << "Nodos explorados (dijkstra): " << cont << endl;

	Path pathInverse;
	Vector2D current = pend;
	pathInverse.points.push_back(current);
	while (current != pinit && current != NULL) {

		current = getPrevious(cFrom, current);
		if (current != NULL) pathInverse.points.push_back(current);

	}

	cFrom.clear();

	int size = pathInverse.points.size();
	Path path;
	for (int i = 0; i < size; i++) {
		path.points.push_back(pathInverse.points.back());
		pathInverse.points.pop_back();
	}

	return path;

}

float heuristic(Vector2D a, Vector2D b) {
	//# Manhattan distance on a square grid
	return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool operator<(pair<float, Vector2D> a, pair<float, Vector2D> b) { return a.first > b.first ? true : false; }

Path Agent::greedyBestFirstSearch(Vector2D pinit, Vector2D pend, Graph terrain) {

	priority_queue<pair<float, Vector2D>> frontier;
	frontier.push(make_pair(0, pinit));
	vector<Connection> cFrom;
	cFrom.push_back(Connection(NULL, pinit, 0));

	int cont = 0; //Contador de nodos explorados

	while (frontier.size() > 0) {

		pair<float, Vector2D> current = frontier.top();
		frontier.pop();
		if (current.second == pend) break; //Early exit

		vector<Connection> neighboor = terrain.getConnections(&current.second);
		cont++;
		
		for (int i = 0; i < neighboor.size(); i++) {
			if (!search(cFrom, *neighboor[i].getToNode())) {
				float heu = heuristic(*neighboor[i].getToNode(), pend);
				frontier.push(make_pair(heu, *neighboor[i].getToNode()));
				cFrom.push_back(neighboor[i]);
			}
		}

	}

	cout << "Nodos explorados (greedyBestFirstSearch): " << cont << endl;

	Path pathInverse;
	Vector2D current = pend;
	pathInverse.points.push_back(current);
	while (current != pinit && current != NULL) {

		current = getPrevious(cFrom, current);
		if (current != NULL) pathInverse.points.push_back(current);

	}

	cFrom.clear();

	int size = pathInverse.points.size();
	Path path;
	for (int i = 0; i < size; i++) {
		path.points.push_back(pathInverse.points.back());
		pathInverse.points.pop_back();
	}

	return path;

}

Path Agent::apuntero(Vector2D pinit, Vector2D pend, Graph terrain) {

	priority_queue<pair<float, Vector2D>> frontier;
	frontier.push(make_pair(0, pinit));
	vector<Connection> cFrom;
	cFrom.push_back(Connection(NULL, pinit, 0));

	int cont = 0; //Contador de nodos explorados

	while (frontier.size() > 0) {

		pair<int, Vector2D> current = frontier.top();
		frontier.pop();
		//if (current.second == pend) break; //Early exit

		vector<Connection> neighboor = terrain.getConnections(&current.second);
		cont++;

		for (int i = 0; i < neighboor.size(); i++) {
			int newCost = current.first + neighboor[i].cost;
			bool isthere = search(cFrom, *neighboor[i].getToNode());
			float heu = newCost + (heuristic(*neighboor[i].getToNode(), pend) / (CELL_SIZE * CELL_SIZE));
			if (!isthere) {
				neighboor[i].cost = newCost;
				frontier.push(make_pair(heu, *neighboor[i].getToNode()));
				cFrom.push_back(neighboor[i]);
			}
			else {
				int pos = find(cFrom, *neighboor[i].getToNode());
				int provaCost = cFrom[pos].cost;
				if (newCost < provaCost) {
					neighboor[i].cost = newCost;
					frontier.push(make_pair(heu, *neighboor[i].getToNode()));
					cFrom[pos] = neighboor[i];
				}
			}
		}

	}

	cout << "Nodos explorados (apuntero): " << cont << endl;

	Path pathInverse;
	Vector2D current = pend;
	pathInverse.points.push_back(current);
	while (current != pinit && current != NULL) {

		current = getPrevious(cFrom, current);
		if (current != NULL) pathInverse.points.push_back(current);

	}

	cFrom.clear();

	int size = pathInverse.points.size();
	Path path;
	for (int i = 0; i < size; i++) {
		path.points.push_back(pathInverse.points.back());
		pathInverse.points.pop_back();
	}

	return path;

}