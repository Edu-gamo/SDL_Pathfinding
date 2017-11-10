#include "Agent.h"
#include <queue>

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
Path Agent::pathFind(Vector2D pinit, Vector2D pend, int method, Graph terrain) {

	pinit.x = (int)pinit.x;
	pinit.y = (int)pinit.y;

	switch (method) {
		case 0://Breadth First Search
			return breadthFirstSearch(pinit, pend, terrain);
			break;
		case 1://Dijkstra

			break;
		case 2://Greedy Best-First-Search

			break;
		case 3://A*

			break;
	}

}

bool search(vector<pair<Vector2D, Vector2D>> cameFrom, Vector2D position) {
	bool find = false;
	int i = 0;
	while (!find && i < cameFrom.size()) {
		if (cameFrom[i].first == position) find = true;
		i++;
	}
	return find;
}

bool search(vector<Connection> cameFrom, Vector2D position) {
	bool find = false;
	int i = 0;
	while (!find && i < cameFrom.size()) {
		if (*cameFrom[i].getToNode() == position) find = true;
		i++;
	}
	return find;
}

Vector2D getPrevious(vector<pair<Vector2D, Vector2D>> cameFrom, Vector2D position) {
	bool find = false;
	Vector2D prev = NULL;
	int i = 0;
	while (!find && i < cameFrom.size()) {
		if (cameFrom[i].first == position) {
			find = true;
			prev = cameFrom[i].second;
		}
		i++;
	}
	return prev;
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
	//vector<pair<Vector2D, Vector2D>> cameFrom;
	vector<Connection> cFrom;
	//cameFrom.push_back(make_pair(pinit, NULL));
	cFrom.push_back(Connection(NULL, pinit, 0));

	while (frontier.size() > 0){

		Vector2D current = frontier.front();
		frontier.pop();
		
		vector<Connection> neighboor = terrain.getConnections(&current);

		for (int i = 0; i < neighboor.size(); i++) {
			/*if (!search(cameFrom, *neighboor[i].getToNode())) {
				frontier.push(*neighboor[i].getToNode());
				cameFrom.push_back(make_pair(*neighboor[i].getToNode(), current));
			}*/
			if (!search(cFrom, *neighboor[i].getToNode())) {
				frontier.push(*neighboor[i].getToNode());
				//cFrom.push_back(Connection(*neighboor[i].getToNode(), current, neighboor[i].getCost()));
				cFrom.push_back(neighboor[i]);
			}
		}

	}

	Path pathInverse;
	Vector2D current = pend;
	pathInverse.points.push_back(current);
	while (current != pinit && current != NULL) {

		//current = getPrevious(cameFrom, current);
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