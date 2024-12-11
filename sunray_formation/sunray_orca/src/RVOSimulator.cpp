#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace RVO 
{
	// 设置默认代理和KD树
RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
{
	kdTree_ = new KdTree(this);
}

RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
{
	kdTree_ = new KdTree(this);
	defaultAgent_ = new Agent(this);

	defaultAgent_->maxNeighbors_ = maxNeighbors;
	defaultAgent_->maxSpeed_ = maxSpeed;
	defaultAgent_->neighborDist_ = neighborDist;
	defaultAgent_->radius_ = radius;
	defaultAgent_->timeHorizon_ = timeHorizon;
	defaultAgent_->timeHorizonObst_ = timeHorizonObst;
	defaultAgent_->velocity_ = velocity;
}

RVOSimulator::~RVOSimulator()
{
	if (defaultAgent_ != NULL) {
		delete defaultAgent_;
	}

	for (size_t i = 0; i < agents_.size(); ++i) {
		delete agents_[i];
	}

	for (size_t i = 0; i < obstacles_.size(); ++i) {
		delete obstacles_[i];
	}

	delete kdTree_;
}
// 指定代理的初始位置和运动参数
size_t RVOSimulator::addAgent(const Vector2 &position)
{
	if (defaultAgent_ == NULL) {
		return RVO_ERROR;
	}

	Agent *agent = new Agent(this);

	agent->position_ = position;
	agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
	agent->maxSpeed_ = defaultAgent_->maxSpeed_;
	agent->neighborDist_ = defaultAgent_->neighborDist_;
	agent->radius_ = defaultAgent_->radius_;
	agent->timeHorizon_ = defaultAgent_->timeHorizon_;
	agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
	agent->velocity_ = defaultAgent_->velocity_;

	agent->id_ = agents_.size();	// 从1开始

	agents_.push_back(agent);

	return agents_.size() - 1;
}

size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
{
	Agent *agent = new Agent(this);

	agent->position_ = position;
	agent->maxNeighbors_ = maxNeighbors;
	agent->maxSpeed_ = maxSpeed;
	agent->neighborDist_ = neighborDist;
	agent->radius_ = radius;
	agent->timeHorizon_ = timeHorizon;
	agent->timeHorizonObst_ = timeHorizonObst;
	agent->velocity_ = velocity;

	agent->id_ = agents_.size();

	agents_.push_back(agent);

	return agents_.size() - 1;
}
// 添加障碍物
size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
{
	if (vertices.size() < 2) {
		return RVO_ERROR;
	}

	const size_t obstacleNo = obstacles_.size();

	for (size_t i = 0; i < vertices.size(); ++i) {
		Obstacle *obstacle = new Obstacle();
		obstacle->point_ = vertices[i];

		if (i != 0) {
			obstacle->prevObstacle_ = obstacles_.back();
			obstacle->prevObstacle_->nextObstacle_ = obstacle;
		}

		if (i == vertices.size() - 1) {
			obstacle->nextObstacle_ = obstacles_[obstacleNo];
			obstacle->nextObstacle_->prevObstacle_ = obstacle;
		}

		obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

		if (vertices.size() == 2) {
			obstacle->isConvex_ = true;
		}
		else {
			obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
		}

		obstacle->id_ = obstacles_.size();

		obstacles_.push_back(obstacle);
	}

	return obstacleNo;
}
// 执行避障算法:计算每个代理的新速度
void RVOSimulator::computeVel()
{
	kdTree_->buildAgentTree();

	for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
	{
		agents_[i]->computePrefVelocity();
		agents_[i]->computeNeighbors();
		agents_[i]->computeNewVelocity();
	}

	
	for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
	{
		agents_[i]->update();
	}

	globalTime_ += timeStep_;
}

size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
{
	return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
}

size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
{
	return agents_[agentNo]->maxNeighbors_;
}

float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
{
	return agents_[agentNo]->maxSpeed_;
}

float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
{
	return agents_[agentNo]->neighborDist_;
}

size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
{
	return agents_[agentNo]->agentNeighbors_.size();
}

size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
{
	return agents_[agentNo]->obstacleNeighbors_.size();
}

size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
{
	return agents_[agentNo]->orcaLines_.size();
}

size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
{
	return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
}

const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
{
	return agents_[agentNo]->orcaLines_[lineNo];
}

const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
{
	return agents_[agentNo]->position_;
}

const Vector2 &RVOSimulator::getAgentGoal(size_t agentNo) const
{
	return agents_[agentNo]->goal_;
}

const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
{
	return agents_[agentNo]->prefVelocity_;
}

float RVOSimulator::getAgentRadius(size_t agentNo) const
{
	return agents_[agentNo]->radius_;
}

float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
{
	return agents_[agentNo]->timeHorizon_;
}

float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
{
	return agents_[agentNo]->timeHorizonObst_;
}

const Vector2 &RVOSimulator::getAgentVelCMD(size_t agentNo) const
{
	return agents_[agentNo]->newVelocity_;
}

const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
{
	return agents_[agentNo]->velocity_;
}

float RVOSimulator::getGlobalTime() const
{
	return globalTime_;
}

size_t RVOSimulator::getNumAgents() const
{
	return agents_.size();
}

size_t RVOSimulator::getNumObstacleVertices() const
{
	return obstacles_.size();
}

const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
{
	return obstacles_[vertexNo]->point_;
}

size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
{
	return obstacles_[vertexNo]->nextObstacle_->id_;
}

size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
{
	return obstacles_[vertexNo]->prevObstacle_->id_;
}

float RVOSimulator::getTimeStep() const
{
	return timeStep_;
}

void RVOSimulator::processObstacles()
{
	kdTree_->buildObstacleTree();
}

bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
{
	return kdTree_->queryVisibility(point1, point2, radius);
}

// 设置代理参数
void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
{
	if (defaultAgent_ == NULL) {
		defaultAgent_ = new Agent(this);
	}

	defaultAgent_->maxNeighbors_ = maxNeighbors;
	defaultAgent_->maxSpeed_ = maxSpeed;
	defaultAgent_->neighborDist_ = neighborDist;
	defaultAgent_->radius_ = radius;
	defaultAgent_->timeHorizon_ = timeHorizon;
	defaultAgent_->timeHorizonObst_ = timeHorizonObst;
	defaultAgent_->velocity_ = velocity;
}

void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
{
	agents_[agentNo]->maxNeighbors_ = maxNeighbors;
}

void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
{
	agents_[agentNo]->maxSpeed_ = maxSpeed;
}

void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
{
	agents_[agentNo]->neighborDist_ = neighborDist;
}

void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
{
	agents_[agentNo]->position_ = position;
}

void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2 &prefVelocity)
{
	agents_[agentNo]->prefVelocity_ = prefVelocity;
}

void RVOSimulator::setAgentGoal(size_t agentNo, const Vector2 &Goal)
{
	agents_[agentNo]->goal_ = Goal;
}

void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
{
	agents_[agentNo]->radius_ = radius;
}

void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
{
	agents_[agentNo]->timeHorizon_ = timeHorizon;
}

void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
{
	agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
}

void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
{
	agents_[agentNo]->velocity_ = velocity;
}

void RVOSimulator::setTimeStep(float timeStep)
{
	timeStep_ = timeStep;
}
}
