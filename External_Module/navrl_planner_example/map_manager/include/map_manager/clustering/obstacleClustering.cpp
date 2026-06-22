/*
	FILE: obstacleClustering.cpp
	------------------------
	function implementation of static obstacle clustering
*/

#include <map_manager/clustering/obstacleClustering.h>

obstacleClustering::obstacleClustering(double res){
	this->res_ = res;
}


void obstacleClustering::run(const std::vector<Eigen::Vector3d>& localCloud){
	// run DBSCAN clustering
	this->runDBSCAN(localCloud, this->initialClusters_);

	// run KMeans clustering for each DBSCAN cluster
	std::vector<pointCluster> refinedCluster = this->initialClusters_;
	std::vector<bool> refineComplete (int(refinedCluster.size()), false);
	std::vector<bboxVertex> rotatedInitialBBoxes;
	for (int level=0; level<this->treeLevel_; ++level){
		std::vector<pointCluster> newRefinedCluster;
		std::vector<bool> newRefinedComplete;
		for (int n=0; n<int(refinedCluster.size()); ++n){
			if (not refineComplete[n]){
				// if the refinement is not completed, find the bbox orientation and density
				bboxVertex vertex;
				double density = this->getOrientation(refinedCluster[n], vertex);
				rotatedInitialBBoxes.push_back(vertex);
				// if the density is greater than threshold, mark it as complete
				if (density >= this->densityThresh_){
					newRefinedCluster.push_back(refinedCluster[n]);
					newRefinedComplete.push_back(true);
				}
				else{ // else split it into two
					std::vector<pointCluster> subCloudClusters;
					this->runKmeans(refinedCluster[n], subCloudClusters);
					// evaluate new clusters
					// if got improvement, add new clusters into the refined bbox array
					std::vector<bboxVertex> subVertices;
					double avgDensity = 0.0;
					for (int i=0; i<int(subCloudClusters.size()); ++i){
						bboxVertex subVertex;
						double subDensity = this->getOrientation(subCloudClusters[i], subVertex);
						avgDensity += subDensity/int(subCloudClusters.size());
						subVertices.push_back(subVertex);
					}
					// if (avgDensity/density > this->improveThresh_){
					for (int i=0; i<int(subCloudClusters.size()); ++i){
						newRefinedCluster.push_back(subCloudClusters[i]);
						newRefinedComplete.push_back(false);
					}
					// }
					// else{
					// 	newRefinedCluster.push_back(refinedCluster[n]);
					// 	newRefinedComplete.push_back(true);
					// }
				}
			}
			else{
				newRefinedCluster.push_back(refinedCluster[n]);
				newRefinedComplete.push_back(true);
			}
		}
		refinedCluster = newRefinedCluster;
		refineComplete = newRefinedComplete;
	}

	std::vector<bboxVertex> refinedRotatedBBoxes;
	for (int i=0; i<int(refinedCluster.size()); ++i){
		bboxVertex vertex;
		this->getOrientation(refinedCluster[i], vertex);
		refinedRotatedBBoxes.push_back(vertex);
	}

	this->refinedRotatedBBoxes_ = refinedRotatedBBoxes;
	this->rotatedInitialBBoxes_ = rotatedInitialBBoxes;
}

void obstacleClustering::runDBSCAN(const std::vector<Eigen::Vector3d>& localCloud, std::vector<pointCluster>& cloudClusters){
	std::vector<Point> points;
	for (int i=0; i<int(localCloud.size()); ++i){
		points.push_back(Point (localCloud[i](0), localCloud[i](1), localCloud[i](2)));
	}

	this->dbscan_.reset(new DBSCAN (this->eps_, this->minPts_, points));
	this->dbscan_->run();


	std::vector<vector<int>> cluster = this->dbscan_->getCluster();
	cloudClusters.clear();
	for (int i=0; i<int(cluster.size()); ++i){ // number of cluster
		Eigen::Vector3d clusterMin = localCloud[cluster[i][0]];
		Eigen::Vector3d clusterMax = localCloud[cluster[i][0]];
		std::vector<Eigen::Vector3d> cloudCluster;
		for (int j=0; j<int(cluster[i].size()); ++j){
			Eigen::Vector3d p = localCloud[cluster[i][j]];
			cloudCluster.push_back(p);

			if (p(0) < clusterMin(0)){
				clusterMin(0) = p(0);
			}
			else if (p(0) > clusterMax(0)){
				clusterMax(0) = p(0);
			}

			if (p(1) < clusterMin(1)){
				clusterMin(1) = p(1);
			}
			else if (p(1) > clusterMax(1)){
				clusterMax(1) = p(1);
			}

			if (p(2) < clusterMin(2)){
				clusterMin(2) = p(2);
			}
			else if (p(2) > clusterMax(2)){
				clusterMax(2) = p(2);
			}
		}
		Eigen::Vector3d centroid = (clusterMin + clusterMax)/2.0;
		cloudClusters.push_back(pointCluster (cloudCluster, i, centroid, clusterMin, clusterMax));
	}
}

void obstacleClustering::runKmeans(const pointCluster& cluster, std::vector<pointCluster>& cloudClusters){
	int dimension = 3;
	int clusterNum = 2;
	
	// initialize centroid
	// find the point furthest from the cloud centroid
	double** data = new double*[int(cluster.points.size())];
	double maxDist = 0.0;
	Eigen::Vector3d fpoint;
	for (int i=0; i<int(cluster.points.size()); ++i){
		Eigen::Vector3d diff = cluster.points[i] - cluster.centroid;
		// diff(2) = 0; 
		double dist = diff.norm();

		if (dist > maxDist){
			maxDist = dist;
			fpoint = cluster.points[i];
		}

		// convert data
		data[i] = new double[dimension];
		data[i][0] = cluster.points[i](0);
		data[i][1] = cluster.points[i](1);
		data[i][2] = cluster.points[i](2);
	}

	// find the furthest point from the above point
	maxDist = 0.0;
	Eigen::Vector3d ffpoint;
	for (int i=0; i<int(cluster.points.size()); ++i){
		Eigen::Vector3d diff = cluster.points[i] - fpoint;
		// diff(2) = 0;
		double dist = diff.norm();
		if (dist > maxDist){
			maxDist = dist;
			ffpoint = cluster.points[i];
		}
	}
	std::vector<Eigen::Vector3d> initPoints {fpoint, ffpoint};
	// std::vector<Eigen::Vector3d> initPoints {fpoint, cluster.centroid};
	// run kmeans-----------------------------------------
	this->kmeans_.reset(new KMeans (dimension, clusterNum));

	// init centroid
	this->kmeans_->centroid = new double*[clusterNum];
	for (int i=0; i<clusterNum; ++i){
		this->kmeans_->centroid[i] = new double[dimension];
		this->kmeans_->centroid[i][0] = initPoints[i](0);
		this->kmeans_->centroid[i][1] = initPoints[i](1);
		this->kmeans_->centroid[i][2] = initPoints[i](2);
	}

	for (int i=0; i<this->kmeansIterNum_; ++i){
		this->kmeans_->Cluster(int(cluster.points.size()), data);
	}

	// retrive results
	cloudClusters.resize(clusterNum);
	for (int i=0; i<int(cluster.points.size()); ++i){
		int clusterIdx = this->kmeans_->Classify(data[i]); 
		cloudClusters[clusterIdx].id = clusterIdx;
		cloudClusters[clusterIdx].points.push_back(cluster.points[i]);
	}

	for (int i=0; i<int(cloudClusters.size()); ++i){
		Eigen::Vector3d clusterMin = cloudClusters[i].points[0];
		Eigen::Vector3d clusterMax = cloudClusters[i].points[0];
		for (int j=0; j<int(cloudClusters[i].points.size()); ++j){
			Eigen::Vector3d p = cloudClusters[i].points[j];

			if (p(0) < clusterMin(0)){
				clusterMin(0) = p(0);
			}
			else if (p(0) > clusterMax(0)){
				clusterMax(0) = p(0);
			}

			if (p(1) < clusterMin(1)){
				clusterMin(1) = p(1);
			}
			else if (p(1) > clusterMax(1)){
				clusterMax(1) = p(1);
			}

			if (p(2) < clusterMin(2)){
				clusterMin(2) = p(2);
			}
			else if (p(2) > clusterMax(2)){
				clusterMax(2) = p(2);
			}
		}
		cloudClusters[i].centroid = (clusterMin + clusterMax)/2.0;
		cloudClusters[i].clusterMin = clusterMin;
		cloudClusters[i].clusterMax = clusterMax;
	}

	delete data;
}

double obstacleClustering::getOrientation(const pointCluster& cluster, bboxVertex& vertex){
	double maxDensity = 0.0;
	double bestAngle = 0.0;
	bboxVertex bestVertex;

	for (int i=0; i<this->angleDiscreteNum_; ++i){
		double angle = M_PI * double(i)/double(this->angleDiscreteNum_);
		Eigen::Matrix3d rot;
		rot << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0,  0, 0, 1;
		Eigen::Vector3d rotClusterMin = cluster.points[0];
		Eigen::Vector3d rotClusterMax = cluster.points[0];		
		for (int n=0; n<int(cluster.points.size()); ++n){
			Eigen::Vector3d p = cluster.points[n];
			Eigen::Vector3d pNew = rot * (p - cluster.centroid) + cluster.centroid;
			
			if (pNew(0) < rotClusterMin(0)){
				rotClusterMin(0) = pNew(0);
			}
			else if (pNew(0) > rotClusterMax(0)){
				rotClusterMax(0) = pNew(0);
			}

			if (pNew(1) < rotClusterMin(1)){
				rotClusterMin(1) = pNew(1);
			}
			else if (pNew(1) > rotClusterMax(1)){
				rotClusterMax(1) = pNew(1);
			}

			if (pNew(2) < rotClusterMin(2)){
				rotClusterMin(2) = pNew(2);
			}
			else if (pNew(2) > rotClusterMax(2)){
				rotClusterMax(2) = pNew(2);
			}		
		}

		bboxVertex currVertex (rotClusterMin, rotClusterMax, -angle);
		double lengthNum = (rotClusterMax(0) - rotClusterMin(0))/this->res_ + 1;
		double widthNum = (rotClusterMax(1) - rotClusterMin(1))/this->res_ + 1;
		double heightNum = (rotClusterMax(2) - rotClusterMin(2))/this->res_ + 1;
		double density = double(cluster.points.size())/(lengthNum * widthNum * heightNum);

		if (density > maxDensity){
			maxDensity = density;
			bestAngle = angle;
			bestVertex = currVertex;
		}
	}

	// rotate the vertex 
	for (int i=0; i<int(bestVertex.vert.size()); ++i){
		Eigen::Matrix3d rot;
		rot << cos(-bestAngle), -sin(-bestAngle), 0, sin(-bestAngle), cos(-bestAngle), 0, 0, 0, 1;
		Eigen::Vector3d v = bestVertex.vert[i];
		Eigen::Vector3d newVertex = rot * (v - cluster.centroid) + cluster.centroid;
		bestVertex.vert[i] = newVertex;
	}
	vertex = bestVertex;
	return maxDensity;
}


std::vector<pointCluster> obstacleClustering::getInitialCluster(){
	return this->initialClusters_;
}

std::vector<bboxVertex> obstacleClustering::getRotatedInitialBBoxes(){
	return this->rotatedInitialBBoxes_;
}

std::vector<bboxVertex> obstacleClustering::getRefinedBBoxes(){
	return this->refinedRotatedBBoxes_;
}

std::vector<staticObstacle> obstacleClustering::getStaticObstacles(){
	std::vector<staticObstacle> staticObstacles;
	for (int i=0; i<int(this->refinedRotatedBBoxes_.size()); ++i){
		staticObstacle sob;
		sob.centroid = this->refinedRotatedBBoxes_[i].centroid;
		sob.size = this->refinedRotatedBBoxes_[i].dimension;
		sob.yaw = this->refinedRotatedBBoxes_[i].angle;
		staticObstacles.push_back(sob);
	}
	return staticObstacles;
}