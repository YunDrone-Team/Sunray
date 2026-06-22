/*
	FILE: ESDFMap.h
	--------------------------------------
	definition of ESDF map
*/
#include <map_manager/ESDFMap.h>

namespace mapManager{
	ESDFMap::ESDFMap(){
		this->ns_ = "esdf_map";
		this->hint_ = "[ESDFMap]";
	}

	ESDFMap::ESDFMap(const ros::NodeHandle& nh){
		this->ns_ = "esdf_map";
		this->hint_ = "[ESDFMap]";
		this->nh_ = nh;
		this->initParam();
		this->initESDFParam();
		this->registerPub();
		this->registerESDFPub();
		this->registerCallback();
		this->registerESDFCallback();
	}

	void ESDFMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->initESDFParam();
		this->registerPub();
		this->registerESDFPub();
		this->registerCallback();
		this->registerESDFCallback();
	}

	void ESDFMap::initESDFParam(){
		int reservedSize = this->occupancy_.size();
		this->esdfTemp1_.resize(reservedSize, 0);
		this->esdfTemp2_.resize(reservedSize, 0);
		this->esdfDistancePos_.resize(reservedSize, 10000);
		this->esdfDistanceNeg_.resize(reservedSize, 10000);
		this->esdfDistance_.resize(reservedSize, 10000);
	}

	void ESDFMap::registerESDFPub(){
		this->esdfPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/esdf", 10);
	}

	void ESDFMap::registerESDFCallback(){
		this->esdfTimer_ = this->nh_.createTimer(ros::Duration(0.05), &ESDFMap::updateESDFCB, this);
		this->esdfPubTimer_ = this->nh_.createTimer(ros::Duration(0.05), &ESDFMap::ESDFPubCB, this);
	}

	void ESDFMap::updateESDFCB(const ros::TimerEvent& ){
		if (not this->esdfNeedUpdate_){
			return;
		}
		ros::Time startTime, endTime;
		startTime = ros::Time::now();
		this->updateESDF3D();
		endTime = ros::Time::now();
		if (this->verbose_){
			cout << this->hint_ << ": ESDF update time: " << (endTime - startTime).toSec() << " s." << endl; 
		}
		this->esdfNeedUpdate_ = false;

	}

	void ESDFMap::updateESDF3D(){
		Eigen::Vector3i minRange = this->localBoundMin_;
		Eigen::Vector3i maxRange = this->localBoundMax_;

		// positive DT
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
		  		this->fillESDF([&](int z){return this->occupancyInflated_[this->indexToAddress(x, y, z)] == true ? 0 : std::numeric_limits<double>::max();},
		      			 [&](int z, double val) {this->esdfTemp1_[this->indexToAddress(x, y, z)] = val;}, minRange(2), maxRange(2), 2);
			}
		}

		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int z=minRange(2); z<=maxRange(2); ++z){
				this->fillESDF([&](int y){return this->esdfTemp1_[this->indexToAddress(x, y, z)];},
					     [&](int y, double val){this->esdfTemp2_[this->indexToAddress(x, y, z)]=val;}, minRange(1), maxRange(1), 1);
			}
		}

		for (int y=minRange(1); y<=maxRange(1); ++y){
			for (int z=minRange(2); z<=maxRange(2); ++z){
		  		this->fillESDF([&](int x) {return this->esdfTemp2_[this->indexToAddress(x, y, z)];},
		           		 	   [&](int x, double val){this->esdfDistancePos_[indexToAddress(x, y, z)]=this->mapRes_ * std::sqrt(val);}, minRange(0), maxRange(0), 0);
			}
		}

		// negative DT
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
				this->fillESDF([&](int z){return this->occupancyInflated_[this->indexToAddress(x, y, z)] == false ? 0 : std::numeric_limits<double>::max();},
		  		[&](int z, double val){this->esdfTemp1_[this->indexToAddress(x, y, z)]=val;}, minRange(2), maxRange(2), 2);
			}
		}

		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int z=minRange(2); z<=maxRange(2); ++z){
				this->fillESDF([&](int y) {return this->esdfTemp1_[this->indexToAddress(x, y, z)];},
		       				   [&](int y, double val){this->esdfTemp2_[this->indexToAddress(x, y, z)]=val;}, minRange(1), maxRange(1), 1);
			}
		}

		for (int y=minRange(1); y<=maxRange(1); ++y){
			for (int z=minRange(2); z<=maxRange(2); ++z){
				this->fillESDF([&](int x){return this->esdfTemp2_[this->indexToAddress(x, y, z)];},
		       				   [&](int x, double val){this->esdfDistanceNeg_[this->indexToAddress(x, y, z)]=this->mapRes_ * std::sqrt(val);}, minRange(0), maxRange(0), 0);
			}
		}

		// combine positive and negative DT
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
				for (int z=minRange(2); z<=maxRange(2); ++z){
					int idx = this->indexToAddress(x, y, z);
					this->esdfDistance_[idx] = this->esdfDistancePos_[idx];

					if (this->esdfDistanceNeg_[idx]>0.0){
				  		this->esdfDistance_[idx] += (-this->esdfDistanceNeg_[idx] + this->mapRes_);
					}
				}
			}
		}
	}

	template <typename F_get_val, typename F_set_val>
	void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim){
		int v[this->mapVoxelMax_(dim)];
		double z[this->mapVoxelMax_(dim) + 1];

		int k = start;
		v[start] = start;
		z[start] = -std::numeric_limits<double>::max();
		z[start + 1] = std::numeric_limits<double>::max();

		for (int q = start + 1; q <= end; q++) {
			k++;
			double s;

			do {
				k--;
				s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
			}while (s <= z[k]);

			k++;

			v[k] = q;
			z[k] = s;
			z[k + 1] = std::numeric_limits<double>::max();
		}

		k = start;

		for (int q = start; q <= end; q++) {
			while (z[k + 1] < q) k++;
			double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
			f_set_val(q, val);
		}		
	}

	double ESDFMap::getDistance(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->getDistance(idx);
	}

	double ESDFMap::getDistance(const Eigen::Vector3i& idx){
		Eigen::Vector3i idx1 = idx;
		this->boundIndex(idx1);
		return this->esdfDistance_[this->indexToAddress(idx1)];
	}

	inline double ESDFMap::getDistanceTrilinear(const Eigen::Vector3d& pos){
		if (not this->isInMap(pos)){
			return 0;
		}
		Eigen::Vector3d posMinus = pos - 0.5 * this->mapRes_ * Eigen::Vector3d::Ones();
		Eigen::Vector3i idxMinus;
		Eigen::Vector3d idxMinusPos, diff;
		this->posToIndex(posMinus, idxMinus);
		this->indexToPos(idxMinus, idxMinusPos);

		diff = (pos - idxMinusPos) * this->mapRes_;

		double values[2][2][2];
		for (int x=0; x<2; ++x){
			for (int y=0; y<2; ++y){
				for (int z=0; z<2; ++z){
					Eigen::Vector3i currIdx = idxMinus + Eigen::Vector3i(x, y, z);
					values[x][y][z] = this->getDistance(currIdx);
				}
			}
		}

		// interpolation for distance
		double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
		double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
		double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
		double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
		double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
		double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
		double dist = (1 - diff[2]) * v0 + diff[2] * v1;
		return dist;		
	}

	inline double ESDFMap::getDistanceWithGradTrilinear(const Eigen::Vector3d& pos, Eigen::Vector3d& grad){
		if (not this->isInMap(pos)){
			return 0;
		}
		Eigen::Vector3d posMinus = pos - 0.5 * this->mapRes_ * Eigen::Vector3d::Ones();
		Eigen::Vector3i idxMinus;
		Eigen::Vector3d idxMinusPos, diff;
		this->posToIndex(posMinus, idxMinus);
		this->indexToPos(idxMinus, idxMinusPos);

		diff = (pos - idxMinusPos) * this->mapRes_;

		double values[2][2][2];
		for (int x=0; x<2; ++x){
			for (int y=0; y<2; ++y){
				for (int z=0; z<2; ++z){
					Eigen::Vector3i currIdx = idxMinus + Eigen::Vector3i(x, y, z);
					values[x][y][z] = this->getDistance(currIdx);
				}
			}
		}

		// interpolation for distance
		double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
		double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
		double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
		double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
		double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
		double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
		double dist = (1 - diff[2]) * v0 + diff[2] * v1;

		// intepolation for gradient
		double mapResInv = 1.0/this->mapRes_;
		grad(2) = (v1 - v0) * mapResInv;
		grad(1) = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mapResInv;
		grad(0) = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
		grad(0) += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
		grad(0) += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
		grad(0) += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
		grad(0) *= mapResInv;

		return dist;
	}

	void ESDFMap::ESDFPubCB(const ros::TimerEvent& ){
		this->publishESDF();
	}

	void ESDFMap::publishESDF(){
		double dist;
		pcl::PointCloud<pcl::PointXYZI> cloud;
		pcl::PointXYZI pt;

		const double minDist = 0.0;
		const double maxDist = 5.0;

		Eigen::Vector3i minRange = this->localBoundMin_;
		Eigen::Vector3i maxRange = this->localBoundMax_;
		this->boundIndex(minRange);
		this->boundIndex(maxRange);

		Eigen::Vector3d pos;
		double height = this->position_(2);
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
				this->indexToPos(Eigen::Vector3i(x, y, 1), pos);
				pos(2) = height;

				dist = this->getDistance(pos);
				dist = std::min(dist, maxDist);
				dist = std::max(dist, minDist);

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = height;
				pt.intensity = (dist - minDist)/(maxDist - minDist);
				cloud.push_back(pt);
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(cloud, cloud_msg);

		this->esdfPub_.publish(cloud_msg);		
	}
}