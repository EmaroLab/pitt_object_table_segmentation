/**
 * @file sphere_segmentation_srv.cpp
 * **File** <br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; src/segmentation_services/sphere_segmentation_srv.cpp
 * @author Buoncompagni Luca, </p>
 * **Contacts** <br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <mailto:luca.buoncompagni@edu.unige.it>
 * @date May 10, 2015 </p>
 * **Institution** <br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; DIBRIS, emaroLAB, University of Genoa.
 * @version 2.1
 *
 * @brief The ROS service, to **identify a sphere** in a point cloud.
 *
 * -------------------------------
 * **Programm behaviour** </p>
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
 * This implementation uses RANSAC methods to identify a sphere in a point cloud.
 * Its inputs and outputs messages are implmented in the <a href="https://github.com/EmaroLab/pitt_msgs">pitt_msgs</a> package,
 * at the file:
 * @code pitt_msgs/srv/PrimitiveSegmentation.srv @endcode
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
 * In particular, inputs are:
 * - \c sensor_msgs/PointCloud2 \c cloud    &nbsp;&nbsp; the cloud in which search for a sphere,
 * - \c sensor_msgs/PointCloud2 \c normals  &nbsp;&nbsp; the normal vector for all the points of the cloud.
 * While input parameters are:
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
 * On the other hand, outputs are:
 * -
 * Finally, also outup parameters are provided with the value of that parameter used during computation.
 * This is identified by the suffix \c used_
 * (e.g., )
 *
 * @see Point Cloud Library (PCL) <a href="http://docs.pointclouds.org/1.7.0/group__sample__consensus.html">RANSAC</a> API
 * @see ransac_segmentation
 */


// #######################################################
// #   DEPENDENCIES
// #######################################################

// for pcl and ros
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
// for my message
#include "pitt_msgs/PrimitiveSegmentation.h"
// for my static library
#include "../point_cloud_library/pc_manager.h"
#include "../point_cloud_library/srv_manager.h"

using namespace pcm;
using namespace pcl;
using namespace std;
using namespace srvm;
using namespace pitt_msgs;

ros::NodeHandle* nh_ptr = NULL;

// default param names
static const double SPHERE_NORMAL_DISTANCE_WEIGTH = 0.001; //0.0001;
static const double SPHERE_DISTANCE_TH = 0.007; // 0.7;
static const double SPHERE_MIN_RADIUS_LIMIT = 0.005;
static const double SPHERE_MAX_RADIUS_LIMIT = 0.500;
static const int SPHERE_MAX_ITERATION_LIMIT = 1000; //20;
static const double SPHERE_EPS_ANGLE_TH = 0.0;
static const double SPHERE_MIN_OPENING_ANGLE_DEGREE = 100.0; // degree
static const double SPHERE_MAX_OPENING_ANGLE_DEGREE = 180.0; // degree

// call Euclidean Cluster Extraction (ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
bool ransacSphereDetection(PrimitiveSegmentation::Request &req, PrimitiveSegmentation::Response &res){

	// get input points
	PCLCloudPtr cloud = PCManager::cloudForRosMsg( req.cloud); 		// input cloud
	PCLNormalPtr normals = PCManager::normForRosMsg( req.normals);	// input norms

    // initialize input parameters
	int maxIterations;
	double normalDistanceWeight, distanceThreshold, minRadiusLimit, maxRadiusLimit, epsAngleTh, minOpeningAngle, maxOpeningAngle;

	// get params or set to default values
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_NORMAL_DISTANCE_WEIGHT,
				  normalDistanceWeight, SPHERE_NORMAL_DISTANCE_WEIGTH);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_DISTANCE_TH,
				  distanceThreshold, SPHERE_DISTANCE_TH);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MAX_ITERATION_LIMIT,
                  maxIterations, SPHERE_MAX_ITERATION_LIMIT);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MIN_RADIUS_LIMIT,
				  minRadiusLimit, SPHERE_MIN_RADIUS_LIMIT);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MAX_RADIUS_LIMIT,
				  maxRadiusLimit, SPHERE_MAX_RADIUS_LIMIT);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_EPS_ANGLE_TH,
				  epsAngleTh, SPHERE_EPS_ANGLE_TH);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MIN_OPENING_ANGLE_DEGREE,
                  minOpeningAngle, SPHERE_MIN_OPENING_ANGLE_DEGREE);
	nh_ptr->param(srvm::PARAM_NAME_SPHERE_MAX_OPENING_ANGLE_DEGREE,
                  maxOpeningAngle, SPHERE_MAX_OPENING_ANGLE_DEGREE);

	// apply RANSAC
	SACSegmentationFromNormals< PointXYZ, Normal> seg;
	ModelCoefficients::Ptr coefficients_sphere( new ModelCoefficients);
	PointIndices::Ptr inliers_sphere( new PointIndices);
	seg.setOptimizeCoefficients( true);
	seg.setModelType( SACMODEL_SPHERE);
	seg.setMethodType( SAC_RANSAC);
	seg.setNormalDistanceWeight( normalDistanceWeight);
	seg.setMaxIterations( maxIterations);
	seg.setDistanceThreshold( distanceThreshold);
	seg.setRadiusLimits( minRadiusLimit, maxRadiusLimit);
	seg.setInputCloud( cloud);
	seg.setInputNormals( normals);
	seg.setEpsAngle( epsAngleTh);
	seg.setMinMaxOpeningAngle( minOpeningAngle / 180.0  * M_PI, maxOpeningAngle / 180.0 * M_PI);
	// Obtain the sphere inliers and coefficients
	seg.segment( *inliers_sphere, *coefficients_sphere);

	// set returning value
	res.inliers = PCManager::inlierToVectorMsg( inliers_sphere);	// inlier w.r.t. the input cloud
	res.coefficients = PCManager::coefficientToVectorMsg( coefficients_sphere);
	// set returning center of mass (= the center of the sphere)
	if( coefficients_sphere->values.size() > 0){
		res.x_centroid = coefficients_sphere->values[ 0];
		res.y_centroid = coefficients_sphere->values[ 1];
		res.z_centroid = coefficients_sphere->values[ 2];
		ROS_INFO(" estimated sphere centroid: %f  %f  %f", coefficients_sphere->values[ 0], coefficients_sphere->values[ 1], coefficients_sphere->values[ 2]);
	}

	// coeff 0:centreX, 1:centreY, 2:centreZ, 3:radious
//	if( inliers_sphere->indices.size() > 0)
//		cout << " sphere found ... inliers:" << inliers_sphere->indices.size() <<
//				" Cx:" << coefficients_sphere->values[ 0] <<
//				" Cy:" << coefficients_sphere->values[ 1] <<
//				" Cz:" << coefficients_sphere->values[ 2] <<
//				" radius:" << coefficients_sphere->values[ 3] << endl;
//	else cout << " NO sphere found" << endl;

	return true;
}


// Initialize node
int main(int argc, char **argv){
	ros::init(argc, argv, srvm::SRV_NAME_RANSAC_SPHERE_FILTER);
	ros::NodeHandle nh;
	nh_ptr = &nh;

	ros::ServiceServer service = nh.advertiseService(srvm::SRV_NAME_RANSAC_SPHERE_FILTER, ransacSphereDetection);
	ros::spin();

	return 0;
}
