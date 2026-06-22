
#include <navigation_runner/safeAction.h>

namespace safeAction {
	Agent::Agent(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "safe_action";
		this->hint_ = "[SafeAction]";
		this->initParam();
		// get dynamic obstacle service
		this->getSafeActionServer_ = this->nh_.advertiseService("rl_navigation/get_safe_action", &Agent::getSafeAction, this);
		this->getSafeActionMapServer_ = this->nh_.advertiseService("rl_navigation/get_safe_action_map", &Agent::getSafeActionMap, this);
		this->staticObsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("rl_navigation/safe_action_static_obstacles", 10);
		this->staticObsVisTimer_ = this->nh_.createTimer(ros::Duration(0.05), &Agent::staticObsVisCallback, this);
		this->dynObsVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("rl_navigation/safe_action_dynamic_obstacles", 10);
		this->dynObsVisTimer_ = this->nh_.createTimer(ros::Duration(0.05), &Agent::dynObsVisCallback, this);
	}

	void Agent::initParam(){
		// prediction horizon
		if (not this->nh_.getParam(this->ns_ + "/time_horizon", this->timeHorizon_)){
			this->timeHorizon_ = 3.0;
			cout << this->hint_ << ": No prediction time horizon param. Use default: 3.0s" << endl;
		}
		else{
			cout << this->hint_ << ": Prediction time horizon is set to: " << this->timeHorizon_ << endl;
		}		

		// time step
		if (not this->nh_.getParam(this->ns_ + "/time_step", this->timeStep_)){
			this->timeStep_ = 0.05;
			cout << this->hint_ << ": No time step param. Use default: 0.05s" << endl;
		}
		else{
			cout << this->hint_ << ": Time step is set to " << this->timeStep_ << endl;
		}
		
		// minimum height
		if (not this->nh_.getParam(this->ns_ + "/min_height", this->minHeight_)){
			this->minHeight_ = 0.5;
			cout << this->hint_ << ": No min height param. Use default: 0.5m" << endl;
		}
		else{
			cout << this->hint_ << ": Min height is set to " << this->minHeight_ << endl;
		}

		// maximum height
		if (not this->nh_.getParam(this->ns_ + "/max_height", this->maxHeight_)){
			this->maxHeight_ = 1.5;
			cout << this->hint_ << ": No max height param. Use default: 1.5m" << endl;
		}
		else{
			cout << this->hint_ << ": Max height is set to " << this->maxHeight_ << endl;
		}

		// safety distance
		if (not this->nh_.getParam(this->ns_ + "/safety_distance", this->maxHeight_)){
			this->safetyDist_ = 0.3;
			cout << this->hint_ << ": No safety distance param. Use default: 0.3m" << endl;
		}
		else{
			cout << this->hint_ << ": Safety distance is set to " << this->safetyDist_ << endl;
		}

		// use safety shield when there are static obstacles
		if (not this->nh_.getParam(this->ns_ + "/use_safety_in_static", this->useSafetyInStatic_)){
			this->useSafetyInStatic_ = false;
			cout << this->hint_ << ": No use safety in static param. Use default: false" << endl;
		}
		else{
			cout << this->hint_ << ": Use safety in static is set to " << this->useSafetyInStatic_ << endl;
		}
	}

	Plane Agent::getORCAPlane(const Vector3& agentPos, const Vector3& agentVel, double agentRadius,
						      const Vector3& obsPos, const Vector3& obsVel, double obsRadius, bool useCircle, bool& inVO){
		const double invTimeHorizon = 1.0 / this->timeHorizon_;
		/* Create agent ORCA planes. */
		const Vector3 relativePosition = obsPos - agentPos;
		const Vector3 relativeVelocity = agentVel - obsVel;
		const double distSq = absSq(relativePosition);
		const double combinedRadius = agentRadius + obsRadius;
		const double combinedRadiusSq = sqr(combinedRadius);

		Plane plane;
		Vector3 u;
		if (distSq > combinedRadiusSq){
			/* No collision. */
			const Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;
			/* Vector from cutoff center to relative velocity. */
			const double wLengthSq = absSq(w);

			const double dotProduct = w * relativePosition;

			const double angleVO = asin(combinedRadius/abs(relativePosition));
			const double distToObs = abs((relativeVelocity * this->timeHorizon_) - relativePosition);
			const double angleVel = acos(relativePosition * relativeVelocity / (abs(relativePosition) * abs(relativeVelocity)));
			if (useCircle){
				if (dotProduct < 0.0f && sqr(dotProduct) > combinedRadiusSq * wLengthSq){
					// not in VO
					if (distToObs < combinedRadius){
						inVO = true;
					}

					/* Project on cut-off circle. */
					const double wLength = std::sqrt(wLengthSq);
					const Vector3 unitW = w / wLength;

					plane.normal = unitW;
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else{
					if (angleVel < angleVO){
						inVO = true;
					}
					/* Project on cone. */
					const double a = distSq;
					const double b = relativePosition * relativeVelocity;
					const double c = absSq(relativeVelocity) - absSq(cross(relativePosition, relativeVelocity)) / (distSq - combinedRadiusSq);
					const double t = (b + std::sqrt(sqr(b) - a * c)) / a;
					const Vector3 w = relativeVelocity - t * relativePosition;
					const double wLength = abs(w);
					const Vector3 unitW = w / wLength;

					plane.normal = unitW;
					u = (combinedRadius * t - wLength) * unitW;
				}
			}
			else{
				if (angleVel < angleVO){
					inVO = true;
				}
				/* Project on cone. */
				const double a = distSq;
				const double b = relativePosition * relativeVelocity;
				const double c = absSq(relativeVelocity) - absSq(cross(relativePosition, relativeVelocity)) / (distSq - combinedRadiusSq);
				const double t = (b + std::sqrt(sqr(b) - a * c)) / a;
				const Vector3 w = relativeVelocity - t * relativePosition;
				const double wLength = abs(w);
				const Vector3 unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedRadius * t - wLength) * unitW;
			}
		}
		else{
			/* Collision. */
			const double invTimeStep = 1.0f / this->timeStep_;
			const Vector3 w = relativeVelocity - invTimeStep * relativePosition;
			const double wLength = abs(w);
			const Vector3 unitW = w / wLength;

			plane.normal = unitW;
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}

		plane.point = agentVel + 1.0f * u;
		return plane;
	}

	bool Agent::getSafeActionMap(navigation_runner::GetSafeActionMap::Request& req,
								 navigation_runner::GetSafeActionMap::Response& res){
		this->staticObsPosVec_.clear();
		this->staticObsSizeVec_.clear();
		this->dynObsPosVec_.clear();
		this->dynObsSizeVec_.clear();		


		// Load position
		Vector3 agentPos = Vector3(req.agent_position.x, req.agent_position.y, req.agent_position.z);

		// // Load velocity
		// Vector3 agentVel = Vector3(req.agent_velocity.x, req.agent_velocity.y, req.agent_velocity.z);

		// Load prefered velocity
		Vector3 preferredVel = Vector3(req.rl_velocity.x, req.rl_velocity.y, req.rl_velocity.z);

		// Agent size
		double agentRadius = req.agent_size;

		// init new velocity
		Vector3 safeVel = preferredVel;		

		// maximum velocity
		double maxVel = req.max_velocity;

		// ORCA planes
		std::vector<Plane> orcaPlane;
		bool needSolve = false;
		// dynamic obstacles
		for (int i=0; i<int(req.dynamic_obs_position.size()); ++i){
			// double xySize = std::max(req.dynamic_obs_size[i].x, req.dynamic_obs_size[i].y);
			double obsRadius = std::sqrt(pow(req.dynamic_obs_size[i].x, 2) + pow(req.dynamic_obs_size[i].y, 2)) / 2.;
			int numCubes = std::ceil(req.dynamic_obs_size[i].z / (obsRadius/2.));
			double zRemain = req.dynamic_obs_size[i].z -  (numCubes-1) * (obsRadius/2);
			double zCenterStart = req.dynamic_obs_position[i].z - req.dynamic_obs_size[i].z/2. + zRemain/2.;
			
			for (int n=0; n<numCubes; ++n){
				Vector3 obsPos = Vector3(req.dynamic_obs_position[i].x, req.dynamic_obs_position[i].y, zCenterStart + n*obsRadius/2.);
				Vector3 obsVel = Vector3(req.dynamic_obs_velocity[i].x, req.dynamic_obs_velocity[i].y, req.dynamic_obs_velocity[i].z);
				const bool useCircle = true;
				bool inVO;
				Plane plane = this->getORCAPlane(agentPos, preferredVel, agentRadius+this->safetyDist_, obsPos, obsVel, obsRadius, useCircle, inVO);
				orcaPlane.push_back(plane);

				this->dynObsPosVec_.push_back(obsPos);
				this->dynObsSizeVec_.push_back(obsRadius);
				if (inVO){
					needSolve = true;
				}
			}
		}

		// static obstacles
		for (int i=0; i<int(req.static_obs_position.size()); ++i){
			double radius = 0.1;
			int numCubesX = std::ceil((req.static_obs_size[i].x/2. - radius) / (radius));
			int numCubesY = std::ceil((req.static_obs_size[i].y/2. - radius) / (radius));
			int numCubesZ = std::ceil((req.static_obs_size[i].z/2. - radius) / (radius));
			
			Vector3 obsPos = Vector3(req.static_obs_position[i].x, req.static_obs_position[i].y, req.static_obs_position[i].z); 
			Vector3 obsVel = Vector3(0, 0, 0);
			double obsRadius = radius;

			Vector3 obsPosCurr;
			Vector3 xDirection = Vector3(cos(req.static_obs_angle[i]), sin(req.static_obs_angle[i]), 0);
			Vector3 yDirection = Vector3(-sin(req.static_obs_angle[i]), cos(req.static_obs_angle[i]), 0);
			Vector3 zDirection = Vector3(0, 0, 1);
			for (int nx=-numCubesX; nx<=numCubesX; ++nx){
				for (int ny=-numCubesY; ny<=numCubesY; ++ny){
					for (int nz=-numCubesZ; nz<=numCubesZ; ++nz){
						obsPosCurr = obsPos + double(nx) * xDirection * obsRadius
										    + double(ny) * yDirection * obsRadius 
										    + double(nz) * zDirection * obsRadius;
						// cout << "static: " << obsPosCurr[0] << " " << obsPosCurr[1] << " " << obsPosCurr[2] << endl; 
						const bool useCircle = true;
						bool inVO;
						Plane plane = this->getORCAPlane(agentPos, preferredVel, this->safetyDist_, obsPosCurr, obsVel, obsRadius, useCircle, inVO);
						orcaPlane.push_back(plane);
						if (inVO){
							needSolve = true;
						}
						this->staticObsPosVec_.push_back(obsPosCurr);
						this->staticObsSizeVec_.push_back(obsRadius);
					}
				}
			}
		}

		// if (not this->useSafetyInStatic_ and hasStatic){
		// 	orcaPlane.clear();
		// }

		// height plane constraint
		if (agentPos[2] + preferredVel[2] * this->timeHorizon_  - this->minHeight_  < 0){
			Plane minHeightPlane;
			minHeightPlane.normal = Vector3(0, 0, 1.);
			minHeightPlane.point = Vector3(0, 0, 0);
			orcaPlane.push_back(minHeightPlane);
		}
		else if (this->maxHeight_ - agentPos[2] - preferredVel[2] * this->timeHorizon_ < 0){
			Plane maxHeightPlane;
			maxHeightPlane.normal = Vector3(0, 0, -1.);
			maxHeightPlane.point = Vector3(0, 0, 0);
			orcaPlane.push_back(maxHeightPlane);
		}

		if (needSolve){
			const size_t planeFail = linearProgram3(orcaPlane, maxVel, preferredVel, false, safeVel);

			if (planeFail < orcaPlane.size()){
				cout << "fail initial: " << planeFail << endl;
				linearProgram4(orcaPlane, planeFail, maxVel, safeVel);
			}
		}

		res.safe_action.x = safeVel[0];
		res.safe_action.y = safeVel[1];
		res.safe_action.z = safeVel[2];

		return true;
	}

	bool Agent::getSafeAction(navigation_runner::GetSafeAction::Request& req, 
						      navigation_runner::GetSafeAction::Response& res){
		this->staticObsPosVec_.clear();
		this->staticObsSizeVec_.clear();
		this->dynObsPosVec_.clear();
		this->dynObsSizeVec_.clear();
		


		// Load position
		Vector3 agentPos = Vector3(req.agent_position.x, req.agent_position.y, req.agent_position.z);

		// // Load velocity
		// Vector3 agentVel = Vector3(req.agent_velocity.x, req.agent_velocity.y, req.agent_velocity.z);

		// Load prefered velocity
		Vector3 preferredVel = Vector3(req.rl_velocity.x, req.rl_velocity.y, req.rl_velocity.z);

		// Agent size
		double agentRadius = req.agent_size;

		// init new velocity
		Vector3 safeVel = preferredVel;		

		// maximum velocity
		double maxVel = req.max_velocity;

		// ORCA planes
		std::vector<Plane> orcaPlane;
		bool needSolve = false;
		// dynamic obstacles
		for (int i=0; i<int(req.obs_position.size()); ++i){
			double xySize = std::max(req.obs_size[i].x, req.obs_size[i].y);
			int numCubes = std::ceil(req.obs_size[i].z / xySize);
			double zRemain = req.obs_size[i].z -  (numCubes-1) * xySize;
			double zCenterStart = req.obs_position[i].z - req.obs_size[i].z/2. + zRemain/2.;
			
			for (int n=0; n<numCubes; ++n){
				Vector3 obsPos = Vector3(req.obs_position[i].x, req.obs_position[i].y, zCenterStart + n*xySize);
				Vector3 obsVel = Vector3(req.obs_velocity[i].x, req.obs_velocity[i].y, req.obs_velocity[i].z);
				double obsRadius = std::sqrt(pow(req.obs_size[i].x, 2) + pow(req.obs_size[i].y, 2)) / 2.;
				const bool useCircle = true;
				bool inVO;
				Plane plane = this->getORCAPlane(agentPos, preferredVel, agentRadius+this->safetyDist_, obsPos, obsVel, obsRadius, useCircle, inVO);
				orcaPlane.push_back(plane);

				this->dynObsPosVec_.push_back(obsPos);
				this->dynObsSizeVec_.push_back(obsRadius);
				if (inVO){
					needSolve = true;
				}
			}
		}

		// static obstacles
		std::vector<Eigen::Vector3d> staticPoints;
		bool hasStatic = false;
		int numsLaser = req.laser_points.size()/3;
		for (int i=0; i<numsLaser; ++i){
			Vector3 laserStop = Vector3(req.laser_points[3*i+0], req.laser_points[3*i+1], req.laser_points[3*i+2]);
			Vector3 obsPos = laserStop;

			// for clustering
			staticPoints.push_back(Eigen::Vector3d (obsPos[0], obsPos[1], obsPos[2]));
		}


		// clustering static obstacles
		std::vector<std::vector<Eigen::Vector3d>> clusters;
		this->getClusters(staticPoints, clusters);
		for (int i=0; i<int(clusters.size()); ++i){
			double minX = clusters[i][0](0);
			double maxX = clusters[i][0](0);
			double minY = clusters[i][0](1);
			double maxY = clusters[i][0](1);
			double minZ = clusters[i][0](2);
			double maxZ = clusters[i][0](2);			
			for (int n=0; n<int(clusters[i].size()); ++n){
				if (minX < clusters[i][n](0)){
					minX = clusters[i][n](0);
				}
				else if (maxX > clusters[i][n](0)){
					maxX = clusters[i][n](0);
				}
				if (minY < clusters[i][n](1)){
					minY = clusters[i][n](1);
				}
				else if (maxY > clusters[i][n](1)){
					maxY = clusters[i][n](1);
				}
				if (minZ < clusters[i][n](2)){
					minZ = clusters[i][n](2);
				}
				else if (maxZ > clusters[i][n](2)){
					maxZ = clusters[i][n](2);
				}
			}

			Eigen::Vector3d center ((minX + maxX)/2., (minY + maxY)/2., (minZ + maxZ)/2.);

			// calculate radius
			double maxDist = 0;
			for (int n=0; n<int(clusters[i].size()); ++n){
				Eigen::Vector3d diff = clusters[i][n] - center;
				diff(2) = 0;
				if (diff.norm() > maxDist){
					maxDist = diff.norm();
				}
			}
			Eigen::Vector3d robotPos (agentPos[0], agentPos[1], agentPos[2]);
			Eigen::Vector3d obsDirection = center - robotPos;
			obsDirection /= obsDirection.norm();
			
			double obsRadius = maxDist * 1.5;
			Eigen::Vector3d obsCenter = center + obsDirection * maxDist;
			Vector3 obsPos = Vector3 (obsCenter(0), obsCenter(1), obsCenter(2));
			Vector3 obsVel = Vector3(0, 0, 0);
			const bool useCircle = true;
			bool inVO;
			Plane plane = this->getORCAPlane(agentPos, preferredVel, 0.0, obsPos, obsVel, obsRadius, useCircle, inVO); 
			orcaPlane.push_back(plane);
			hasStatic = true;
			if (inVO){
				needSolve = true;
			}
			this->staticObsPosVec_.push_back(obsPos);
			this->staticObsSizeVec_.push_back(obsRadius);
		}


		if (not this->useSafetyInStatic_ and hasStatic){
			orcaPlane.clear();
		}

		// height plane constraint
		if (agentPos[2] + preferredVel[2] * this->timeHorizon_  - this->minHeight_  < 0){
			Plane minHeightPlane;
			minHeightPlane.normal = Vector3(0, 0, 1.);
			minHeightPlane.point = Vector3(0, 0, 0);
			orcaPlane.push_back(minHeightPlane);
		}
		else if (this->maxHeight_ - agentPos[2] - preferredVel[2] * this->timeHorizon_ < 0){
			Plane maxHeightPlane;
			maxHeightPlane.normal = Vector3(0, 0, -1.);
			maxHeightPlane.point = Vector3(0, 0, 0);
			orcaPlane.push_back(maxHeightPlane);
		}

		if (needSolve){
			const size_t planeFail = linearProgram3(orcaPlane, maxVel, preferredVel, false, safeVel);

			if (planeFail < orcaPlane.size()){
				linearProgram4(orcaPlane, planeFail, maxVel, safeVel);
			}
		}

		res.safe_action.x = safeVel[0];
		res.safe_action.y = safeVel[1];
		res.safe_action.z = safeVel[2];
		return true;
	}

	void Agent::getClusters(const std::vector<Eigen::Vector3d>& points, std::vector<std::vector<Eigen::Vector3d>>& clusters){
		if (points.size() != 0){
			std::vector<onboardDetector::Point> pointsDB;
			for (int i=0; i<int(points.size()); ++i){
				Eigen::Vector3d p = points[i];
				onboardDetector::Point pDB;
				pDB.x = p(0);
				pDB.y = p(1);
				pDB.z = p(2);
				pDB.clusterID = -1;
				pointsDB.push_back(pDB);
			}
			this->dbCluster_.reset(new onboardDetector::DBSCAN (2, 0.2, pointsDB));
			// DBSCAN clustering
			this->dbCluster_->run();

			// get the cluster data with bounding boxes
			// iterate through all the clustered points and find number of clusters
			int clusterNum = 0;
			for (size_t i=0; i<this->dbCluster_->m_points.size(); ++i){
				onboardDetector::Point pDB = this->dbCluster_->m_points[i];
				if (pDB.clusterID > clusterNum){
					clusterNum = pDB.clusterID;
				}
			}

			clusters.clear();
			clusters.resize(clusterNum);
			for (size_t i=0; i<this->dbCluster_->m_points.size(); ++i){
				onboardDetector::Point pDB = this->dbCluster_->m_points[i];
				if (pDB.clusterID > 0){
					Eigen::Vector3d p;
					p(0) = pDB.x;
					p(1) = pDB.y;
					p(2) = pDB.z;
					clusters[pDB.clusterID-1].push_back(p);
				}            
			}			
		}
	}

	void Agent::staticObsVisCallback(const ros::TimerEvent& ){
		visualization_msgs::MarkerArray obsMsg;
		for (int i=0; i<int(this->staticObsPosVec_.size()); ++i){
			visualization_msgs::Marker obsMarker;
			obsMarker.header.frame_id = "map";
			obsMarker.header.stamp = ros::Time::now();
			obsMarker.ns = "safe_action_static_obs";
			obsMarker.id = i;
			obsMarker.type = visualization_msgs::Marker::SPHERE;
			obsMarker.action = visualization_msgs::Marker::ADD;

			obsMarker.pose.position.x = this->staticObsPosVec_[i][0];
			obsMarker.pose.position.y = this->staticObsPosVec_[i][1];
			obsMarker.pose.position.z = this->staticObsPosVec_[i][2];


			obsMarker.scale.x = 2.0 * this->staticObsSizeVec_[i];  // Diameter (2 * radius)
			obsMarker.scale.y = 2.0 * this->staticObsSizeVec_[i];
			obsMarker.scale.z = 2.0 * this->staticObsSizeVec_[i];

			obsMarker.color.r = 1.0;
			obsMarker.color.g = 0.0;
			obsMarker.color.b = 1.0;
			obsMarker.color.a = 0.7;
			obsMarker.lifetime = ros::Duration(0.1);

			obsMsg.markers.push_back(obsMarker);

		}
		this->staticObsVisPub_.publish(obsMsg);			
	}

	void Agent::dynObsVisCallback(const ros::TimerEvent& ){
		visualization_msgs::MarkerArray obsMsg;

		for (int i=0; i<int(this->dynObsPosVec_.size()); ++i){
			visualization_msgs::Marker obsMarker;
			obsMarker.header.frame_id = "map";
			obsMarker.header.stamp = ros::Time::now();
			obsMarker.ns = "safe_action_dyn_obs";
			obsMarker.id = i;
			obsMarker.type = visualization_msgs::Marker::SPHERE;
			obsMarker.action = visualization_msgs::Marker::ADD;

			obsMarker.pose.position.x = this->dynObsPosVec_[i][0];
			obsMarker.pose.position.y = this->dynObsPosVec_[i][1];
			obsMarker.pose.position.z = this->dynObsPosVec_[i][2];


			obsMarker.scale.x = 2.0 * this->dynObsSizeVec_[i];  // Diameter (2 * radius)
			obsMarker.scale.y = 2.0 * this->dynObsSizeVec_[i];
			obsMarker.scale.z = 2.0 * this->dynObsSizeVec_[i];

			obsMarker.color.r = 0.0;
			obsMarker.color.g = 0.0;
			obsMarker.color.b = 1.0;
			obsMarker.color.a = 0.7;
			obsMarker.lifetime = ros::Duration(0.1);

			obsMsg.markers.push_back(obsMarker);

		}
		this->dynObsVisPub_.publish(obsMsg);			
	}
}