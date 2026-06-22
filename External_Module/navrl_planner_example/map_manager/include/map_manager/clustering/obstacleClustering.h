/*
	FILE: obstacleClustering.h
	------------------------
	function header of static obstacle clustering
*/
#ifndef OBSTALCE_CLUSTERING_H
#define OBSTALCE_CLUSTERING_H

#include <map_manager/clustering/DBSCAN.h>
#include <map_manager/clustering/Kmeans.h>
#include <thread>
#include <Eigen/Eigen>
#include <math.h>

struct pointCluster{
	std::vector<Eigen::Vector3d> points;
	int id;
	Eigen::Vector3d centroid;
	Eigen::Vector3d clusterMin, clusterMax;

	pointCluster(){}

	pointCluster(const std::vector<Eigen::Vector3d>& _points, int _id, const Eigen::Vector3d& _centroid, 
		         const Eigen::Vector3d& _clusterMin, const Eigen::Vector3d& _clusterMax){
		points = _points;
		id = _id;
		centroid = _centroid;
		clusterMin = _clusterMin;
		clusterMax = _clusterMax;
	}
};

struct bboxVertex{
	std::vector<Eigen::Vector3d> vert;
	Eigen::Vector3d centroid;
	Eigen::Vector3d dimension;
	double angle;
	bboxVertex(){}

	bboxVertex(const Eigen::Vector3d& pmin, const Eigen::Vector3d& pmax, double rotAngle){
		vert.push_back(Eigen::Vector3d (pmax(0), pmax(1), pmax(2)));
		vert.push_back(Eigen::Vector3d (pmin(0), pmax(1), pmax(2)));
		vert.push_back(Eigen::Vector3d (pmin(0), pmin(1), pmax(2)));
		vert.push_back(Eigen::Vector3d (pmax(0), pmin(1), pmax(2)));
		vert.push_back(Eigen::Vector3d (pmax(0), pmax(1), pmin(2)));
		vert.push_back(Eigen::Vector3d (pmin(0), pmax(1), pmin(2)));
		vert.push_back(Eigen::Vector3d (pmin(0), pmin(1), pmin(2)));
		vert.push_back(Eigen::Vector3d (pmax(0), pmin(1), pmin(2)));

		dimension = pmax - pmin;
		centroid = (pmax + pmin)/2.0;
		angle = rotAngle;
	}
};

struct staticObstacle{
	Eigen::Vector3d centroid;
	Eigen::Vector3d size;
	double yaw;
};

class obstacleClustering{
	private:
		std::shared_ptr<DBSCAN> dbscan_;
		std::shared_ptr<KMeans> kmeans_;
		std::vector<pointCluster> initialClusters_;
		std::vector<bboxVertex> refinedRotatedBBoxes_;
		std::vector<bboxVertex> rotatedInitialBBoxes_;
		double res_;

		// DBSCAN paremeters
		double eps_ = 0.5;
		double minPts_ = 15;

		// Kmeans & Refinement parameters
		int treeLevel_ = 3;
		int angleDiscreteNum_ = 20;
		double densityThresh_ = 0.9;
		double improveThresh_ = 1.1;
		int kmeansIterNum_ = 10;


	public:
		obstacleClustering(double res);
		void setParam();
		void run(const std::vector<Eigen::Vector3d>& localCloud);

		void runDBSCAN(const std::vector<Eigen::Vector3d>& localCloud, std::vector<pointCluster>& cloudClusters);
		void runKmeans(const pointCluster& cluster, std::vector<pointCluster>& cloudClusters);
		double getOrientation(const pointCluster& cluster, bboxVertex& vertex);

		std::vector<pointCluster> getInitialCluster();
		std::vector<bboxVertex> getRotatedInitialBBoxes();
		std::vector<bboxVertex> getRefinedBBoxes();
		std::vector<staticObstacle> getStaticObstacles();
};


#endif