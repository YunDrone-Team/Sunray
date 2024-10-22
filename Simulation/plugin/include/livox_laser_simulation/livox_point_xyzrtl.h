  
/*
 * Created on Sun Aug 15 2021
 *
 * Author: EpsAvlc
 */

#ifndef LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_
#define LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

namespace pcl{
struct LivoxPointXyzrtl{
  PCL_ADD_POINT4D;
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
  EIGEN_ALIGN16;
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPointXyzrtl,  
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, reflectivity, reflectivity)
                                   (uint8_t, tag, tag)
                                   (uint8_t, line, line)
)

#endif