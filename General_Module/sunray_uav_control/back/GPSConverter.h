#ifndef _GPS_H_
#define _GPS_H_

#include <ros/ros.h>
#include <cmath>
#include <iostream>

#define WGS84 1984 //Reference ellipsoid for World Geodetic System 1984
#ifdef WGS84
    #define SEMI_MAJOR_AXIS                 6378137.0               //SemimajorAxis: a (Unit is m)
    #define SEMI_MINOR_AXIS                 6356752.31424518        //SemiminorAxis: b (Unit is m)
    #define FLATTENING                      0.0034                  //Flattening: f=(a-b)/a (扁率)
    #define ONE_MINUS_FLATTENING            0.9966                  //(1-f)
    #define INVERSE_FLATTENING              298.257223563           //Inverse of Flattening: 1/f (扁率的倒数)
    #define ECCENTRICITY                    0.0818191908426215  //First Eccentricity: e=sqrt(f(2-f)) (第一偏心率)
    #define ECCENTRICITY_SQUA               0.0066943799901410  //Square of First Eccentricity: e^2=f(2-f) (第一偏心率的平方)
    #define ECCENTRICITY_P_SQUA             0.0067394967422760  //Square of Second Eccentricity: e_p^2=e^2/(1-e^2) (第二偏心率的平方)
    #define A_MULTIPLY_BY_E_SQUA    42697.672707179970  //a*e^2
    #define B_MULTIPLY_BY_E_P_SQUA  42841.311513313570  //b*e_p^2
#endif

/*****************************************************************
* struct type
*****************************************************************/
typedef struct
{
    double longitude;               // unit: deg
    double latitude;                // unit: deg
    double altitude;                // unit: m
}GpsDataType;



typedef struct
{
    double x_north;                 // unit: m
    double y_east;                  // unit: m
    double z_down;                  // unit: m
}NedDataType;



/*****************************************************************
* class type
*****************************************************************/
class GpsTran
{
public:
    GpsTran();
    GpsTran(double initial_longitude, double initial_latitude, double initial_altitude);
    GpsTran(const GpsDataType& initial_gps);
    bool resetInitialGps(double initial_longitude, double initial_latitude, double initial_altitude);
    bool resetInitialGps(const GpsDataType& initial_gps);
    ~GpsTran();

    bool fromGpsToNed(NedDataType& ned, const GpsDataType& gps);
    void fromNedToGps(GpsDataType& gps, const NedDataType& ned);

    bool initialized_;
    
private:
    typedef struct
    {
        double x;                       // unit: m
        double y;                       // unit: m
        double z;                       // unit: m
    }EcefDataType;

    GpsDataType     initial_gps_;
    EcefDataType    initial_ecef_;

    double sin_initial_longitude_;
    double cos_initial_longitude_;
    double sin_initial_latitude_;
    double cos_initial_latitude_;

    bool valid_initial_gps_;

    bool fromGpsToEcef(EcefDataType& ecef, const GpsDataType&  gps);
    void fromEcefToNed(NedDataType&  ned,  const EcefDataType& ecef);
    void fromNedToEcef(EcefDataType& ecef, const NedDataType&  ned);
    void fromEcefToGps(GpsDataType&  gps,  const EcefDataType& ecef);
};

GpsTran::GpsTran()
{
    initialized_ = false;
}

GpsTran::GpsTran(double initial_longitude, double initial_latitude, double initial_altitude)
{
	initial_gps_.longitude 	= initial_longitude;
	initial_gps_.latitude 	= initial_latitude;
	initial_gps_.altitude 	= initial_altitude;

	sin_initial_longitude_ 	= sin(initial_longitude * M_PI/180);
	cos_initial_longitude_ 	= cos(initial_longitude * M_PI/180);
	sin_initial_latitude_ 	= sin(initial_latitude * M_PI/180);
	cos_initial_latitude_ 	= cos(initial_latitude * M_PI/180);

	valid_initial_gps_ = fromGpsToEcef(initial_ecef_, initial_gps_);

    initialized_ = true;

	//std::cout.flags(ios::fixed);
	//std::cout.precision(9); 
}

GpsTran::GpsTran(const GpsDataType& initial_gps)
{
	initial_gps_.longitude 	= initial_gps.longitude;
	initial_gps_.latitude 	= initial_gps.latitude;
	initial_gps_.altitude 	= initial_gps.altitude;

	sin_initial_longitude_ 	= sin(initial_gps.longitude * M_PI/180);
	cos_initial_longitude_ 	= cos(initial_gps.longitude * M_PI/180);
	sin_initial_latitude_ 	= sin(initial_gps.latitude * M_PI/180);
	cos_initial_latitude_ 	= cos(initial_gps.latitude * M_PI/180);

	valid_initial_gps_ = fromGpsToEcef(initial_ecef_, initial_gps_);

    initialized_ = true;

	//std::cout.flags(ios::fixed);
	//std::cout.precision(9); 
}

bool GpsTran::resetInitialGps(double initial_longitude, double initial_latitude, double initial_altitude)
{
	initial_gps_.longitude 	= initial_longitude;
	initial_gps_.latitude 	= initial_latitude;
	initial_gps_.altitude 	= initial_altitude;

	sin_initial_longitude_ 	= sin(initial_longitude * M_PI/180);
	cos_initial_longitude_ 	= cos(initial_longitude * M_PI/180);
	sin_initial_latitude_ 	= sin(initial_latitude * M_PI/180);
	cos_initial_latitude_ 	= cos(initial_latitude * M_PI/180);

	valid_initial_gps_ = fromGpsToEcef(initial_ecef_, initial_gps_);

	//std::cout.flags(ios::fixed);
	//std::cout.precision(9); 
}

bool GpsTran::resetInitialGps(const GpsDataType& initial_gps)
{
	initial_gps_.longitude 	= initial_gps.longitude;
	initial_gps_.latitude 	= initial_gps.latitude;
	initial_gps_.altitude 	= initial_gps.altitude;

	sin_initial_longitude_ 	= sin(initial_gps.longitude * M_PI/180);
	cos_initial_longitude_ 	= cos(initial_gps.longitude * M_PI/180);
	sin_initial_latitude_ 	= sin(initial_gps.latitude * M_PI/180);
	cos_initial_latitude_ 	= cos(initial_gps.latitude * M_PI/180);

	valid_initial_gps_ = fromGpsToEcef(initial_ecef_, initial_gps_);

    initialized_ = true;

	//std::cout.flags(ios::fixed);
	//std::cout.precision(9); 
}

GpsTran::~GpsTran() {}


bool GpsTran::fromGpsToEcef(EcefDataType& ecef, const GpsDataType&  gps)
{
	/*****************************************************************
	*** input: 
	* gps: GPS data in "deg" unit  
	* ecef: the coordinate of Earth-centered Earth-fixed(ECEF) in "m" unit
	*** output: 
	* transformation status, "true" means transform successfully, "false" means the gps data is out of bounds
	*****************************************************************/
	if(gps.latitude<=-90||gps.latitude>=90||gps.longitude<-180||gps.longitude>180)
	{
		ROS_ERROR("the gps data is out of bounds.");
		ROS_ERROR_STREAM("longitude = "<<gps.longitude<<", latitude = "<<gps.latitude);
		return false;
	}

	double phi_in_radian 	= gps.latitude*M_PI/180;	//unit: rad
	double lambda_in_radian = gps.longitude*M_PI/180;	//unit: rad
	double sin_phi 	 = sin(phi_in_radian);
	double cos_phi 	 = cos(phi_in_radian);
	double sin_lambda = sin(lambda_in_radian);
	double cos_lambda = cos(lambda_in_radian);
	double radius_of_curvature = SEMI_MAJOR_AXIS / sqrt(1-ECCENTRICITY_SQUA*sin_phi*sin_phi); //unit: m
	double rho = (radius_of_curvature*1+gps.altitude)*cos_phi;				//unit: m
	double rhp = (radius_of_curvature*(1-ECCENTRICITY_SQUA)+gps.altitude);	//unit: m
	ecef.x = rho*cos_lambda;
	ecef.y = rho*sin_lambda;
	ecef.z = rhp*sin_phi;
	// std::cout<<"ecef.x="<<ecef.x<<",\tecef.y="<<ecef.y<<",\tecef.z="<<ecef.z<<std::endl;
	return true;
}


void GpsTran::fromEcefToNed(NedDataType&  ned,  const EcefDataType& ecef)
{
	/*****************************************************************
	*** input: 
	* ecef: the coordinate of Earth-centered Earth-fixed(ECEF) in "m" unit
	* ned: the coordinate of local Cartesian North-East-Down(NED) coordinate system, the unit is "m"
	*** output: void
	*****************************************************************/
	double u = ecef.x - initial_ecef_.x;
	double v = ecef.y - initial_ecef_.y;
	double w = ecef.z - initial_ecef_.z;
	double t = u*cos_initial_longitude_ + v*sin_initial_longitude_;
	ned.x_north = -t*sin_initial_latitude_ 	+ w*cos_initial_latitude_;
	ned.y_east  = -u*sin_initial_longitude_ + v*cos_initial_longitude_;
	ned.z_down  = -t*cos_initial_latitude_  - w*sin_initial_latitude_;
	// std::cout<<"x_north="<<ned.x_north<<",\ty_east="<<ned.y_east<<",\tz_down="<<ned.z_down<<std::endl;
}

void GpsTran::fromNedToEcef(EcefDataType& ecef, const NedDataType&  ned)
{
	/*****************************************************************
	*** input: 
	* ned: the coordinate of local Cartesian North-East-Down(NED) coordinate system, the unit is "m"
	* ecef: the coordinate of Earth-centered Earth-fixed(ECEF) in "m" unit
	*** output: void
	*****************************************************************/
	double t = -ned.z_down*cos_initial_latitude_ - ned.x_north*sin_initial_latitude_;
	ecef.x = initial_ecef_.x + 	 	 	t*cos_initial_longitude_ -  ned.y_east*sin_initial_longitude_;
	ecef.y = initial_ecef_.y + 	 	 	t*sin_initial_longitude_ +  ned.y_east*cos_initial_longitude_;
	ecef.z = initial_ecef_.z - ned.z_down*sin_initial_latitude_  + ned.x_north*cos_initial_latitude_;
	// std::cout<<"ecef.x="<<ecef.x<<",\tecef.y="<<ecef.y<<",\tecef.z="<<ecef.z<<std::endl;
}

void GpsTran::fromEcefToGps(GpsDataType&  gps,  const EcefDataType& ecef)
{
	/*****************************************************************
	*** input: 
	* ecef: the coordinate of Earth-centered Earth-fixed(ECEF) in "m" unit
	* gps: GPS data in "deg" unit  
	*** output: void
	*****************************************************************/
	gps.longitude = atan2(ecef.y, ecef.x);
	double rho = sqrt(ecef.x*ecef.x + ecef.y*ecef.y);
	double beta = atan2(ecef.z,ONE_MINUS_FLATTENING*rho);
	double sin_beta = sin(beta);
	double cos_beta = cos(beta);
	gps.latitude = atan2(ecef.z+B_MULTIPLY_BY_E_P_SQUA*sin_beta*sin_beta*sin_beta,rho-A_MULTIPLY_BY_E_SQUA*cos_beta*cos_beta*cos_beta);
	double sin_phi = sin(gps.latitude);
	double cos_phi = cos(gps.latitude);
	double beta_new = atan2(ONE_MINUS_FLATTENING*sin_phi,cos_phi);
	char count = 0;
	while(beta!=beta_new && count<5)
	{
		beta = beta_new;
		sin_beta = sin(beta);
		cos_beta = cos(beta);
		gps.latitude = atan2(ecef.z+B_MULTIPLY_BY_E_P_SQUA*sin_beta*sin_beta*sin_beta,rho-A_MULTIPLY_BY_E_SQUA*cos_beta*cos_beta*cos_beta);
		sin_phi = sin(gps.latitude);
		cos_phi = cos(gps.latitude);
		beta_new = atan2(ONE_MINUS_FLATTENING*sin_phi,cos_phi);
		count = count + 1;
	}
	double radius_of_curvature = SEMI_MAJOR_AXIS / sqrt(1-ECCENTRICITY_SQUA*sin_phi*sin_phi); //m
	gps.altitude = rho*cos_phi + (ecef.z+ECCENTRICITY_SQUA*radius_of_curvature*sin_phi)*sin_phi-radius_of_curvature;
	gps.latitude = gps.latitude*180/M_PI;		//convert from rad to deg
	gps.longitude = gps.longitude*180/M_PI;		//convert from rad to deg
	// std::cout<<"longitude="<<gps.longitude<<",\tlatitude="<<gps.latitude<<",\taltitude="<<gps.altitude<<std::endl;
}


bool GpsTran::fromGpsToNed(NedDataType& ned, const GpsDataType& gps)
{
	/*****************************************************************
	*** input: 
	* gps: GPS data in "deg" unit  
	* ned: the coordinate of local Cartesian North-East-Down(NED) coordinate system, the unit is "m"
	*** output: 
	* transformation status, "true" means transform successfully, "false" means the gps data is out of bounds
	*****************************************************************/
	if(valid_initial_gps_)
	{
		EcefDataType ecef;
		bool flag = fromGpsToEcef(ecef, gps);
		if( !flag )
			return false; 
		else
		{
			fromEcefToNed(ned, ecef);
			return true;
		}
	}
	else
	{
		ROS_ERROR("the initial gps data is out of bounds.");
		ROS_WARN("please reset initial gps with member functions resetInitialGps().");
	}
	
}

void GpsTran::fromNedToGps(GpsDataType& gps, const NedDataType& ned)
{
	/*****************************************************************
	*** input: 
	* gps: GPS data in "deg" unit  
	* ned: the coordinate of local Cartesian North-East-Down(NED) coordinate system, the unit is "m"
	*** output: void
	*****************************************************************/
	EcefDataType ecef;
	fromNedToEcef(ecef, ned);
	fromEcefToGps(gps, ecef);
}

#endif 