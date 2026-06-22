#ifndef SAFE_ACTION_H
#define SAFE_ACTION_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <navigation_runner/utils.h>
#include <navigation_runner/solver.h>
#include <navigation_runner/GetSafeAction.h>
#include <navigation_runner/GetSafeActionMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <onboard_detector/dbscan.h>
using std::cout; using std::endl;
namespace safeAction {
	class Agent{
	private:
		std::string ns_;
		std::string hint_;
		ros::NodeHandle nh_;
		ros::ServiceServer getSafeActionServer_;
		ros::ServiceServer getSafeActionMapServer_;
		ros::Publisher staticObsVisPub_;
		ros::Publisher dynObsVisPub_;
		ros::Timer staticObsVisTimer_;
		ros::Timer dynObsVisTimer_;

        std::shared_ptr<onboardDetector::DBSCAN> dbCluster_;

		double timeHorizon_;
		double timeStep_;
		double minHeight_;
		double maxHeight_;
		double safetyDist_;
		bool useSafetyInStatic_;
		std::vector<Vector3> staticObsPosVec_;
		std::vector<double> staticObsSizeVec_;
		std::vector<Vector3> dynObsPosVec_;
		std::vector<double> dynObsSizeVec_;
		
	public:
		Agent(const ros::NodeHandle& nh);
		void initParam();
		Plane getORCAPlane(const Vector3& agentPos, const Vector3& agentVel, double agentRadius,
						   const Vector3& obsPos, const Vector3& obsVel, double obsRadius, bool useCircle, bool& inVO);
		bool getSafeActionMap(navigation_runner::GetSafeActionMap::Request& req,
							  navigation_runner::GetSafeActionMap::Response& res);
		bool getSafeAction(navigation_runner::GetSafeAction::Request& req, 
						   navigation_runner::GetSafeAction::Response& res);
		void getClusters(const std::vector<Eigen::Vector3d>& points, std::vector<std::vector<Eigen::Vector3d>>& clusters);
		void staticObsVisCallback(const ros::TimerEvent& );
		void dynObsVisCallback(const ros::TimerEvent& );
	};
}
#endif
