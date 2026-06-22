/*
	FILE: ESDFMap.h
	--------------------------------------
	ESDF map header file
*/
#ifndef MAPMANAGER_ESDFMAP
#define MAPMANAGER_ESDFMAP
#include <map_manager/occupancyMap.h>

namespace mapManager{
	class ESDFMap : public occMap{
	private:

	protected:
		ros::Timer esdfTimer_;
		ros::Timer esdfPubTimer_;
		ros::Publisher esdfPub_;

		std::vector<double> esdfTemp1_;
		std::vector<double> esdfTemp2_;
		std::vector<double> esdfDistancePos_;
		std::vector<double> esdfDistanceNeg_;
		std::vector<double> esdfDistance_;

	public:
		ESDFMap(); // empty constructor
		ESDFMap(const ros::NodeHandle& nh);
		void initMap(const ros::NodeHandle& nh);
		void initESDFParam();
		void registerESDFPub();
		void registerESDFCallback();
		void updateESDFCB(const ros::TimerEvent& );
		void updateESDF3D();

		template <typename F_get_val, typename F_set_val>
		void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

		// user function
		double getDistance(const Eigen::Vector3d& pos);
		double getDistance(const Eigen::Vector3i& idx);
		double getDistanceTrilinear(const Eigen::Vector3d& pos); // trilinear interpolation (more accurate)
		double getDistanceWithGradTrilinear(const Eigen::Vector3d& pos, Eigen::Vector3d& grad); // trilinear interpolation

		// visualization
		void ESDFPubCB(const ros::TimerEvent& );
		void publishESDF();
	};
}

#endif
