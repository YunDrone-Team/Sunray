#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0)

// WGS84椭球体参数
#define WGS84_A 6378137.0           // 长半轴(m)
#define WGS84_F (1.0/298.257223563) // 扁率
#define WGS84_E2 (2*WGS84_F - WGS84_F*WGS84_F) // 第一偏心率平方

// 经纬高结构体
typedef struct 
{
    double lat;    // 纬度(度)
    double lon;    // 经度(度)
    double alt;    // 高度(m)
} LLH_Coord;

// 三维直角坐标结构体(ENU坐标系)
typedef struct 
{
    double x; // 东方向(m)
    double y; // 北方向(m)
    double z; // 天方向(m)
} ENU_Coord;

// ECEF地心地固坐标系
typedef struct 
{
    double x; // ECEF X坐标(m)
    double y; // ECEF Y坐标(m)
    double z; // ECEF Z坐标(m)
} ECEF_Coord;

/**
 * @brief 将经纬高转换为ECEF坐标
 * @param llh 经纬高坐标
 * @return ECEF地心地固坐标
 */
ECEF_Coord llh_to_ecef(const LLH_Coord* llh) 
{
    ECEF_Coord ecef;
    double lat_rad = llh->lat * DEG_TO_RAD;
    double lon_rad = llh->lon * DEG_TO_RAD;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);
    
    // 计算卯酉圈曲率半径
    double N = WGS84_A / sqrt(1 - WGS84_E2 * sin_lat * sin_lat);
    
    ecef.x = (N + llh->alt) * cos_lat * cos_lon;
    ecef.y = (N + llh->alt) * cos_lat * sin_lon;
    ecef.z = (N * (1 - WGS84_E2) + llh->alt) * sin_lat;
    
    return ecef;
}

/**
 * @brief 计算从初始点到目标点的ENU坐标
 * @param origin 初始点经纬高坐标
 * @param target 目标点经纬高坐标
 * @param enu 输出的ENU坐标(指针)
 */
void calculate_enu_coordinates(const LLH_Coord* origin, const LLH_Coord* target, ENU_Coord* enu) 
{
    // 参数检查
    if (origin == NULL || target == NULL || enu == NULL) {
        return;
    }
    
    // 将原点坐标转换为ECEF
    ECEF_Coord ecef_origin = llh_to_ecef(origin);
    
    // 将目标点坐标转换为ECEF
    ECEF_Coord ecef_target = llh_to_ecef(target);
    
    // 计算ECEF坐标差
    double dx = ecef_target.x - ecef_origin.x;
    double dy = ecef_target.y - ecef_origin.y;
    double dz = ecef_target.z - ecef_origin.z;
    
    // 计算原点的经纬度(弧度)
    double lat_rad = origin->lat * DEG_TO_RAD;
    double lon_rad = origin->lon * DEG_TO_RAD;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);
    
    // ECEF到ENU的旋转矩阵
    // [ -sin(lon)                  cos(lon)                  0       ]
    // [ -sin(lat)*cos(lon)        -sin(lat)*sin(lon)        cos(lat) ]
    // [  cos(lat)*cos(lon)         cos(lat)*sin(lon)         sin(lat) ]
    
    // 计算ENU坐标
    enu->x = -sin_lon * dx + cos_lon * dy;
    enu->y = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    enu->z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
}

/**
 * @brief 计算从ENU坐标到原点的距离和方位信息
 * @param enu ENU坐标
 * @param distance 总距离(输出参数，可为NULL)
 * @param horizontal_dist 水平距离(输出参数，可为NULL)
 * @param azimuth 方位角(度，从北顺时针，输出参数，可为NULL)
 * @param elevation 仰角(度，输出参数，可为NULL)
 */
void calculate_enu_metrics(const ENU_Coord* enu, double* distance, double* horizontal_dist, 
                          double* azimuth, double* elevation) {
    if (enu == NULL) return;
    
    if (distance != NULL) {
        *distance = sqrt(enu->x * enu->x + enu->y * enu->y + enu->z * enu->z);
    }
    
    if (horizontal_dist != NULL) {
        *horizontal_dist = sqrt(enu->x * enu->x + enu->y * enu->y);
    }
    
    if (azimuth != NULL) {
        double az = atan2(enu->x, enu->y) * (180.0 / M_PI);
        if (az < 0) az += 360.0;
        *azimuth = az;
    }
    
    if (elevation != NULL && horizontal_dist != NULL) {
        double h_dist = sqrt(enu->x * enu->x + enu->y * enu->y);
        *elevation = atan2(enu->z, h_dist) * (180.0 / M_PI);
    } else if (elevation != NULL) {
        double h_dist = sqrt(enu->x * enu->x + enu->y * enu->y);
        *elevation = atan2(enu->z, h_dist) * (180.0 / M_PI);
    }
}