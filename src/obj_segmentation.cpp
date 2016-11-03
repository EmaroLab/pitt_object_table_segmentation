/**
 * @file obj_segmentation.cpp
 * **File** <br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; src/obj_segmentation.cpp
 * @author Buoncompagni Luca, </p>
 * **Contacts** <br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <mailto:luca.buoncompagni@edu.unige.it>
 * @date May 10, 2015 </p>
 * **Institution** <br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; DIBRIS, emaroLAB, University of Genoa.
 * @version 2.1
 *
 * @brief The PITT starting node, to **preprocessing** the Point Cloud for primitive identification.
 *
 * -------------------------------
 * **Programm behaviour** </p>
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
 * This file implements a ROS node that acquires Kinect Point Cloud (PC) from the
 * topic specified through the: srvm::DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC parameter
 * (see #main function for more) and performs preprocessing (on #depthAcquisition callback) with the following order:
 * -# PC downsampling (see PCL documentation for more),
 * -# filtering out far points (see deep_filter_srv.cpp ROS service),
 * -# filtering out points of robotic arms (see arm_filter_srv.cpp ROS service),
 * -# PC transforming w.r.t. robot base frame (see TF for more),
 * -# tables segmentation (see supports_segmentation_srv.cpp ROS service),
 * -# objects segmentation, for each support (see cluster_segmentation_srv.cpp ROS service)
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
 * The results of this computation (for each table) is published into the topic specified through the
 * srvm::TOPIC_OUT_NAME_OBJECT_PERCEPTION parameter. This topic accommodates an array of
 * @code pitt_msgs/msg/InlierCluster.msg @endcode
 * messages (see <a href="https://github.com/EmaroLab/pitt_msgs">pitt_msgs</a> package for more details)
 * that contains, for each recognised object in that table:
 * - \c sensor_msgs/PointCloud2 \c cloud: &nbsp;&nbsp; the cloud containing only the object,
 * - \c int32[] \c inliers: &nbsp;&nbsp; the index of the points of the object w.r.t. the original cloud,
 * - \c float32 \c x_centroid, \c y_centroid, \c z_centroid: &nbsp;&nbsp; the coordinates of the center of mass of the cloud
 * - \c int32 \c shape_id: &nbsp;&nbsp; object identifier. At this stage of the architecture this value is not set (the tracker will do it, see the <a href="https://github.com/EmaroLab/pitt_geometric_tracking">pitt_geometric_tracking
</a> for an example).
 *
 * @see pc_manager.h
 * @see pc_primitive.h
 * @see srv_manager.h
 * @see ransac_segmentation.cpp
 * @see <a href="https://github.com/EmaroLab/pitt_geometric_tracking>pitt_geometric_tracking</a>
 */



// #######################################################
// #   DEPENDENCIES
// #######################################################

// for ROS library
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
// for PITT services
#include "pitt_msgs/DeepFilter.h"
#include "pitt_msgs/SupportSegmentation.h"
#include "pitt_msgs/ClusterSegmentation.h"
#include "pitt_msgs/ArmFilter.h"
// for PITT messages
#include "pitt_msgs/ClustersOutput.h"
// for PITT static library
#include "point_cloud_library/pc_manager.h"
#include "point_cloud_library/srv_manager.h"



// #######################################################
// #   TYPE DEF
// #######################################################

/**
 * \brief a vector of costom messages to describe tables.
 * @see @code pitt_msgs/msg/Support.msg @endcode
 */
typedef vector< pitt_msgs::Support> InlierSupports;
/**
 * \brief a pointer to a #InlierSupports.
 * @see  @code pitt_msgs/msg/Support.msg @endcode
 */
typedef boost::shared_ptr< vector< pitt_msgs::Support> > InlierSupportsPtr;
/**
 * \brief a vector of costom messages to describe objects on a table.
 * @see @code pitt_msgs/msg/InliersCluster.msg @endcode
 */
typedef vector< pitt_msgs::InliersCluster> InlierClusters;
/**
 * \brief a pointer to a #InlierClusters.
 * @see  @code pitt_msgs/msg/InliersCluster.msg @endcode
 */
typedef boost::shared_ptr< vector< pitt_msgs::InliersCluster> > InlierClusterPtr;



// #######################################################
// #   CONSTANTS
// #######################################################

/// the cloud will be not not processed if has less number of points
static const int MIN_POINT_IN_ORIGINAL_CLOUD = 30; // TODO: make it as a parameter



// #######################################################
// #   PARAMETERS FROM MAIN CALL
// #######################################################

/**
 * \brief flag for showing raw cloud (white points) on the visualiser of this node.
 *
 * This is a global input parameter that is settable from launcher
 * and acquired during node initialization (see also #main function: argv[ 2]).
 * <br> It cannot be changed on the fly.
 * <br> It default values is set to: srvm::DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD
 */
bool inputShowOriginalCloud;
/**
 * \brief flag for showing identified tables (brown points) on the visualiser of this node.
 *
 * This is a global input parameter that is settable from launcher
 * and acquired during node initialization (see also #main function: argv[ 3]).
 * <br> It cannot be changed on the fly.
 * <br> It default values is set to: srvm::DEFAULT_INPUT_PARAM_SHOW_SUPPORTS
 */
bool inputShowSupportClouds;
/**
 * \brief flag for showing identified objects (random colors) on the visualiser of this node.
 *
 * This is a global input parameter that is settable from launcher
 * and acquired during node initialization (see also #main function: argv[ 4]).
 * <br> It cannot be changed on the fly.
 * <br> It default values is set to: srvm::DEFAULT_INPUT_PARAM_SHOW_CLUSTERS
 */
bool inputShowClusterClouds;
/**
 * \brief flag for showing identified all the points on the table (orange points) on the visualiser of this node.
 *
 * This is a global input parameter that is settable from launcher
 * and acquired during node initialization (see also #main function: argv[ 5]).
 * <br> It cannot be changed on the fly.
 * <br> It default values is set to: srvm::DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT
 */
bool inputShowObjectOnSupport;
/**
 * \brief file path to lag the centroids of the acquired objects in a table. Set to empty to do not log on file.
 *
 * The data, that will be appended on the file for each acquisition will have the following shape:
 * @code "scan id, support idx, cluster idx, centroid X, centroid Y, centroid Z;\n" @endcode
 *
 * This is a global input parameter that is settable from launcher
 * and acquired during node initialization (see also #main function: argv[ 6]).
 * <br> It cannot be changed on the fly.
 * <br> It default values is set to: srvm::DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE
 */
string centroidLogFilePath;



// #######################################################
// #   GLOBAL PRIVATE VARIABLES
// #######################################################

/// variable to initialise the pcm::PCManager.
pcm::PCManager* manager = new pcm::PCManager( false); // true => (visualize)
/// ros handle passed across all services calls.
ros::NodeHandle* nh_ptr = NULL;
/// the PCL visualiser for this node.<p> It will happear if at least one of: #inputShowOriginalCloud, #inputShowSupportClouds, #inputShowClusterClouds, #inputShowObjectOnSupport; is true.
boost::shared_ptr< visualization::PCLVisualizer> vis; // to visualize cloud
/// the ROS publisher in which the message of a new acquistion will be send. <p> Its name is: srvm::TOPIC_OUT_NAME_OBJECT_PERCEPTION.
ros::Publisher clusterPub;
/// the transformation matrix between the RGB-D sensor and the srvm::PARAM_NAME_INPUT_CLOUD_REFERENCE_FRAME frame (it can be changed on the fly).
Eigen::Matrix4f pclTransform;
/// for a better visualiser updating on parallel thread.
boost::thread vis_thread;
/// for a better visualiser updating during different services calls.
boost::mutex vis_mutex;
/// string that will contains info about deep_filter_srv.cpp service, shown in the visualiser while the parameters are changed on the fly.
string log_str_depth = "Loading...";
/// string that will contains info about support_segmentation_srv.cpp service, shown in the visualiser while the parameters are changed on the fly.
string log_str_supp = "Loading...";
/// so far, it is used only for logging.
long scanId = 0;



// #######################################################
// #   UTILITY FUNCTIONS (called in the node callback)
// #######################################################

/// make the visualisation spinning without lagging acroos multi updating.
void visSpin(){
    while( ! vis->wasStopped()){
        boost::mutex::scoped_lock updateLock( vis_mutex);
        vis->spinOnce(100);
    }
}

/**
 * \brief call deep_filter_srv.cpp ROS service.
 *
 * It removes, from input parameter, all the points further than a threshold on the z-axis of the cloud.
 *
 * @param cloud the PC from which remove all the points further than a threshold.
 * @return false if an error occurs during service call, true otherwise.
 */
bool callDeepFilter( PCLCloudPtr& cloud){

    /// Following, the steeps that this function performs:
    /// - load the threshold parameter: srvm::PARAM_NAME_DEEP_SRV_Z_THRESHOLD from ROS pull. Use srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F as default value,
	float inputDepthThreshold;
	nh_ptr->param( srvm::PARAM_NAME_DEEP_SRV_Z_THRESHOLD, inputDepthThreshold, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);

    /// - update visualiser texts about the deep threshold value,
	if (inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport) {
        boost::mutex::scoped_lock updateLock(vis_mutex);
		log_str_depth = boost::str(boost::format("DEPTH THRESHOLD: %f  (-1 -> default value)")
                                   % inputDepthThreshold);
		vis->updateText(log_str_depth, 10, 520, "log_str_depth");
	}

	/// - initialise deep filter server,
    pitt_msgs::DeepFilter srvDeep;
    ros::ServiceClient clientDeep = nh_ptr->serviceClient< pitt_msgs::DeepFilter>( srvm::SRV_NAME_DEEP_FILTER);

	/// - set input data and parameters,
	srvDeep.request.input_cloud = pcm::PCManager::cloudToRosMsg( cloud);
	srvDeep.request.deep_threshold = inputDepthThreshold;

	/// - call service,
	if( ! clientDeep.call( srvDeep)){
        /// 	- check for error during service calls.
        ROS_ERROR_STREAM( " error on calling service " << clientDeep.getService());
        return false;
	} else {
        /// 	- get the repose and overwrite the cloud, if no errors occur.
        // srv.response.cloud_further; // is not used,
        cloud = pcm::PCManager::cloudForRosMsg( srvDeep.response.cloud_closer);
        return true;
	}
}

/**
 * \brief call arm_filter_srv.cpp ROS service.
 *
 * It removes, from input parameter, all the points that are belonging to a bounding box around the robot arms.
 *
 * @param cloud the PC from which remove all the points of the arms.
 * @return false if an error occurs during service call, true otherwise.
 */
bool callArmFilter( PCLCloudPtr& cloud){

    /// Following, the steeps that this function performs:
    /// - load the baunding box parameters (default values:  srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST):
    /// 	- srvm::PARAM_NAME_ARM_SRV_MIN_FOREARM_BOX,
    /// 	- srvm::PARAM_NAME_ARM_SRV_MAX_FOREARM_BOX,
    /// 	- srvm::PARAM_NAME_ARM_SRV_MIN_ELBOW_BOX,
    /// 	- srvm::PARAM_NAME_ARM_SRV_MAX_ELBOW_BOX,
	vector< float> forearmMinBox, forearmMaxBox, elbowMinBox, elbowMaxBox;
    float arr[1] = {-1};
	vector<float> vec(arr, arr +sizeof(arr)/sizeof(float));
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MIN_FOREARM_BOX, forearmMinBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MAX_FOREARM_BOX, forearmMaxBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MIN_ELBOW_BOX, elbowMinBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MAX_ELBOW_BOX, elbowMaxBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);

    /// - initialise arm filter server,
    ros::ServiceClient clientArm = nh_ptr->serviceClient< pitt_msgs::ArmFilter>( srvm::SRV_NAME_ARM_FILTER);
    pitt_msgs::ArmFilter armFilterSrv;

    /// - set input data and parameters,
	armFilterSrv.request.input_cloud = pcm::PCManager::cloudToRosMsg( cloud);
	armFilterSrv.request.forearm_bounding_box_min_value = forearmMinBox;
	armFilterSrv.request.forearm_bounding_box_max_value = forearmMaxBox;
	armFilterSrv.request.elbow_bounding_box_min_value = elbowMinBox;
	armFilterSrv.request.elbow_bounding_box_max_value = elbowMaxBox;

	/// - call service
	if( ! clientArm.call( armFilterSrv)){
        /// - check for error during service calls.
        ROS_ERROR_STREAM( " error on calling service " << clientArm.getService());
        return false;
    } else {
        /// - get the repose and overwrite the cloud, if no errors occur.
        cloud = pcm::PCManager::cloudForRosMsg( armFilterSrv.response.armless_cloud);
        return true;
    }
}

/**
 * \brief call support_segmentation_srv.cpp ROS service.
 *
 * It detects all the vertical planes in the scene and returns the index of their point w.r.t. to original cloud.
 *
 * @param inputCloud the PC from which retrieve the supports.
 * @param normal the Nolrmal vector of the points of the above cloud.
 * @return a vector of set of index indicating the points of a support w.r.t. to acquired cloud. It returns an empty array if errors occur during service call.
 */
InlierSupportsPtr callSupportFilter( PCLCloudPtr inputCloud, PCLNormalPtr normal){

    /// Following, the steeps that this function performs:
    /// - load the parameters for the server (default values: srvm::DEFAULT_SERVICE_PARAMETER_REQUEST or srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F):
    /// 	- srvm::PARAM_NAME_MIN_ITERATIVE_CLOUD_PERCENTAGE,
    /// 	- srvm::PARAM_NAME_MIN_ITERATIVE_SUPPORT_PERCENTAGE,
    /// 	- srvm::PARAM_NAME_HORIZONTAL_VARIANCE_THRESHOLD,
    /// 	- srvm::PARAM_NAME_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD,
    /// 	- srvm::PARAM_NAME_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT,
    /// 	- srvm::PARAM_NAME_RANSAC_MAX_ITERATION_THRESHOLD,
    /// 	- srvm::PARAM_NAME_HORIZONTAL_AXIS,
    /// 	- srvm::PARAM_NAME_SUPPORT_EDGE_REMOVE_OFFSET,
    vector<float> horizontalAxis, edgeRemoveOffset;
    int minIterativeCloudPercentualSize;
    float minIterativePlanePercentualSize, varianceThHorizontal, ransacDistShapeTh, ransacModelNormalDist, ransacMaxIteration;
    nh_ptr->param( srvm::PARAM_NAME_MIN_ITERATIVE_CLOUD_PERCENTAGE, minIterativeCloudPercentualSize, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_MIN_ITERATIVE_SUPPORT_PERCENTAGE, minIterativePlanePercentualSize, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_HORIZONTAL_VARIANCE_THRESHOLD, varianceThHorizontal, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD, ransacDistShapeTh, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT, ransacModelNormalDist, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_RANSAC_MAX_ITERATION_THRESHOLD, ransacMaxIteration, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST);
    nh_ptr->param( srvm::PARAM_NAME_HORIZONTAL_AXIS, horizontalAxis, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
    nh_ptr->param( srvm::PARAM_NAME_SUPPORT_EDGE_REMOVE_OFFSET, edgeRemoveOffset, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);

    /// - initialise support segmentation server,
    ros::ServiceClient client = nh_ptr->serviceClient< pitt_msgs::SupportSegmentation>( srvm::SRV_NAME_SUPPORT_FILTER);
    pitt_msgs::SupportSegmentation srv;

    /// - set input data and parameters,
    srv.request.input_cloud = pcm::PCManager::cloudToRosMsg( inputCloud);
    srv.request.input_norm = pcm::PCManager::normToRosMsg( normal);

    srv.request.min_iterative_cloud_percentual_size = minIterativeCloudPercentualSize;
    srv.request.min_iterative_plane_percentual_size = minIterativePlanePercentualSize;
    srv.request.variance_threshold_for_horizontal = varianceThHorizontal;
    srv.request.ransac_distance_point_in_shape_threshold = ransacDistShapeTh;
    srv.request.ransac_model_normal_distance_weigth = ransacModelNormalDist;
    srv.request.ransac_max_iteration_threshold = ransacMaxIteration;
    srv.request.horizontal_axis = horizontalAxis;
    srv.request.support_edge_remove_offset = edgeRemoveOffset;


	/// - call service,
	InlierSupportsPtr objs ( new InlierSupports( srv.response.supports_description.size()));
	if( ! client.call( srv))
        /// - check and log for errors,
        ROS_ERROR_STREAM( " error on calling service " << client.getService());
	else
        /// - get the service responce to be returned,
        *objs = srv.response.supports_description;

    /// - update the visualiser texts.
    if (inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport) {
        boost::mutex::scoped_lock updateLock(vis_mutex);
        log_str_supp = boost::str(boost::format("Cloud size %%: %f"
                                                         "\nSupport size %%: %f"
                                                         "\nHorizontal variance eps: %f"
                                                         "\nIn-shape distance eps: %f"
                                                         "\nNormal distance weight: %f"
                                                         "\nMax iterations: %i"
                                                         "\nHorizontal axis: (%f, %f, %f)"
                                                         "\nSupports mask offset: (%f, %f, %f)")
                                    %srv.request.min_iterative_cloud_percentual_size
                                    %srv.request.min_iterative_plane_percentual_size
                                    %srv.request.variance_threshold_for_horizontal
                                    %srv.request.ransac_distance_point_in_shape_threshold
                                    %srv.request.ransac_model_normal_distance_weigth
                                    %srv.request.ransac_max_iteration_threshold
                                    %srv.request.horizontal_axis[0]
                                    %srv.request.horizontal_axis[1]
                                    %srv.request.horizontal_axis[2]
                                    %srv.request.support_edge_remove_offset[0]
                                    %srv.request.support_edge_remove_offset[1]
                                    %srv.request.support_edge_remove_offset[2]);
        vis->updateText(log_str_supp, 10, 70, "log_str_supp");
    }
	return( objs);
}

/**
 * \brief call cluster_segmentation_srv.cpp ROS service.
 *
 * It segments a PC in smaller structure where each of them represents a separated object.
 * The points are also described in terms of index in the original cloud.
 *
 * @param cloud the PC to be segmented in clusters.
 * @return an array of structures that describes separated objects in the cloud. An empty array if errors occur on service call.
 */
InlierClusterPtr callClusterSegmentation( PCLCloudPtr cloud){

    /// Following, the steeps that this function performs:
    /// - load the parameters for the server (default values: srvm::DEFAULT_SERVICE_PARAMETER_REQUEST or srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F):
    /// 	- srvm::PARAM_NAME_CLUSTER_TOLERANCE,
    /// 	- srvm::PARAM_NAME_CLUSTER_MIN_RATE,
    /// 	- srvm::PARAM_NAME_CLUSTER_MAX_RATE,
    /// 	- srvm::PARAM_NAME_CLUSTER_TOLERANCE,
    float tollerance, minClusterSizeRate, maxClusterSizeRate;
    int minInputSize;
    nh_ptr->param( srvm::PARAM_NAME_CLUSTER_TOLERANCE, tollerance, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_CLUSTER_MIN_RATE, minClusterSizeRate, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_CLUSTER_MAX_RATE, maxClusterSizeRate, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param( srvm::PARAM_NAME_CLUSTER_TOLERANCE, minInputSize, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST);

    /// - initialise claster segmentation server,
    ros::ServiceClient client = nh_ptr->serviceClient< pitt_msgs::ClusterSegmentation>( srvm::SRV_NAME_CUSTER_FILTER);
    pitt_msgs::ClusterSegmentation srv;

    /// - set input data and parameters,
	srv.request.cloud = pcm::PCManager::cloudToRosMsg( cloud);

    srv.request.tollerance = tollerance;
    srv.request.min_rate = minClusterSizeRate;
    srv.request.max_rate = maxClusterSizeRate;
    srv.request.min_input_size = minInputSize;

    /// - call service,
	InlierClusterPtr inc ( new InlierClusters( srv.response.cluster_objs.size()));
	if( ! client.call( srv))
        /// - check and log for errors,
        ROS_ERROR_STREAM( " error on calling service " << client.getService());
	else
        /// - get the service responce to be returned.
        *inc = srv.response.cluster_objs;

	return( inc);
}



// #######################################################
// #   NODE CALLBACK
// #######################################################

/**
 * \brief Node callback.
 *
 * It is called as soon as a new PC acquisition is available from the sensor.
 * It processes the clud based on ROS services:
 * -# deep_filter_srv.cpp
 * -# arm_filter_srv.cpp
 * -# supports_segmentation_srv.cpp
 * -# cluster_segmentation_srv.cpp
 *
 * and publishes the output message if a successfully acquisition occurs.
 *
 * @param input the Point Cloud message published on the input topic.
 * @see main
 * @see callDeepFilter
 * @see callArmFilter
 * @see callSupportFilter
 * @see callClusterSegmentation
 */
void depthAcquisition( const PointCloud2Ptr& input){

    string centroidFileLog = ""; // initialise for logging on file if path is specified

    /// Following, the steeps that this function performs:
	/// - get Kinect PC inputs as a standard pcl cloud,
	PCLCloudPtr rawCloud = pcm::PCManager::cloudForRosMsg( input);

	/// - compute down-sampling (pcm::PCManager::downSampling),
	PCLCloudPtr cloud = pcm::PCManager::downSampling( rawCloud); //using default DOWN_SAMPLING_RATE

	/// - call deep filter server (#callDeepFilter),
	if( callDeepFilter( cloud)){

		/// - if success: call arm filter server (#callArmFilter),
		if( callArmFilter( cloud)){

			/// - if success: transform the cloud w.r.t. #pclTransform,
			PCLCloudPtr worldCloud( new PCLCloud);
			pcl::transformPointCloud( *cloud, *worldCloud, pclTransform);

            /// - stop if too few points are remaining (w.r.t. #MIN_POINT_IN_ORIGINAL_CLOUD). Otherwise ...
			if( worldCloud->points.size() > MIN_POINT_IN_ORIGINAL_CLOUD){
				/// - compute the normals off all the points,
				PCLNormalPtr normal = pcm::PCManager::estimateNormal( worldCloud); // using default ESTIMATE_NORMAL_SPAN
				/// - show original cloud as gray points, if #inputShowOriginalCloud is true,
				if( inputShowOriginalCloud) {
                    boost::mutex::scoped_lock lock(vis_mutex);
                    pcm::PCManager::updateVisor(vis, worldCloud, normal, 220, 220, 220, "original");
                }

				/// - call the support segmentation service (#callSupportFilter),
				InlierSupportsPtr supports = callSupportFilter( worldCloud, normal);

                /// - for all the detected supports ...
				if( supports->size() > 0){ // at least one support
					for( int i = 0; i < supports->size(); i++){ // for all the found supports
                        /// - show supports as brown points, if #inputShowSupportClouds is true,
						if( inputShowSupportClouds){
							// get horizontal plane from service response
							PCLCloudPtr support = pcm::PCManager::cloudForRosMsg( (* supports)[ i].support_cloud);
							// show points with brown colors
                            boost::mutex::scoped_lock lock(vis_mutex);
                            pcm::PCManager::updateVisor( vis, support, 102, 55, 55,  "table" +
                                    boost::lexical_cast<std::string>( i));
						}
						/// - show cloud of the object on the horizontal plane, as orange points, if #inputShowObjectOnSupport is true,
						PCLCloudPtr onSupport = pcm::PCManager::cloudForRosMsg( (* supports)[ i].on_support_cloud);
						if( inputShowObjectOnSupport) {
                            boost::mutex::scoped_lock lock(vis_mutex);
                            pcm::PCManager::updateVisor(vis, onSupport, 255, 183, 131, "object" +
                                    boost::lexical_cast<std::string>(i));
                        }

                        /// - call cluster segmentation service (#callClusterSegmentation),
						InlierClusterPtr clusters = callClusterSegmentation( onSupport);

						/// - if there is at least one cluster, prepare node output message containing all the claster information for a support,
						boost::shared_ptr< pitt_msgs::ClustersOutput> out ( new pitt_msgs::ClustersOutput);
						if( clusters->size() > 0){ // at least one cluster
							for( int j = 0; j < clusters->size(); j++){ // for all the clusters
                                pitt_msgs::InliersCluster clusterObject = (* clusters)[ j];
								// append this cluster to output
								out->cluster_objs.push_back( clusterObject);

								// get cluster
								PCLCloudPtr clusterCloud = pcm::PCManager::cloudForRosMsg(clusterObject.cloud);

                                /// - show cloud of the segmented objects, with random colors, if #inputShowClusterClouds is true,
								if( inputShowClusterClouds) { // visualize cluster
                                    boost::mutex::scoped_lock lock(vis_mutex);
                                    pcm::PCManager::updateVisor(vis, clusterCloud, "clusterPlane" +
                                                                              boost::lexical_cast<std::string>(j));
                                }

								/// - prepare detached clusters center of mass for logging,
								centroidFileLog += boost::lexical_cast<std::string>(scanId) + ", " +
                                        boost::lexical_cast<std::string>( i) + ", " +
                                        boost::lexical_cast<std::string>( j) + ", " +
                                        boost::lexical_cast<std::string>( clusterObject.x_centroid) + ", " +
                                        boost::lexical_cast<std::string>( clusterObject.y_centroid) + ", " +
                                        boost::lexical_cast<std::string>( clusterObject.z_centroid) + ";\n";
							}
							/// - publish the center of mass of the detached clusters for a specific support,
							clusterPub.publish( out);
                            // TODO: you may modify this publishing to manage multiple clustering on all detected supports in a single message.
						}
					}
				}
			}
		}
	}
	/// - print on screen the results,
	ROS_INFO_STREAM( "raw clusters data: [scan id, support idx, cluster idx, centroid X, cenntroid Y, centroid Z;\\n]" << endl << centroidFileLog);
	/// - if #centroidLogFilePath is not empty, print the same results on file.
    pcm::PCManager::writeToFile( centroidFileLog, centroidLogFilePath, true);
	scanId += 1;
}



// #######################################################
// #   NODE INITIALISER (create callback features)
// #######################################################

/**
 * \brief It initialise this ROS node and spin in order to activate the #depthAcquisition callback as soon as new data is available.
 * It also listens and updates the #pclTransform matrix.
 *
 * @param argv the value of the input parameters, respectively:
 * -# the name of the input topic (containing a PointCloud2) (default value: srvm::DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC),
 * -# the flag for showing the original cloud (see #inputShowOriginalCloud),
 * -# the flag for showing the supports cloud (see #inputShowSupportClouds),
 * -# the flag for showing the clusters cloud (see #inputShowClusterClouds),
 * -# the flag for showing the cloud of the object above the supports (see #inputShowObjectOnSupport),
 * -# the file path in which store the logging information about the segmented cluster centroids (see #centroidLogFilePath).
 * @param argc the number of input parameters + 1 (defaults: executable name). It has to be equal to 7.
 *
 * @return 0.
 */
int main(int argc, char **argv){

    /// Following, the steeps that this function performs:
	/// - instantiate the ROS node with name "obj_segmentation",
	string nodeName = "obj_segmentation";
	int numberOfInputParameter = 6;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle node;
    nh_ptr = &node;

	/// - read and configure input parameters,
	std::string inputPointCloudTopicName;

    if( argc == numberOfInputParameter + 1){
		// args[ 0] is the path to the executable file

		// read the name of the input cloud topic
		inputPointCloudTopicName = srvm::getStringPtrParameter( argv[ 1], srvm::DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC);

		// read the flags to show point cloud plots
		inputShowOriginalCloud = srvm::getBoolPtrParameter( argv[ 2], srvm::DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD);
		inputShowSupportClouds = srvm::getBoolPtrParameter( argv[ 3], srvm::DEFAULT_INPUT_PARAM_SHOW_SUPPORTS);
		inputShowClusterClouds = srvm::getBoolPtrParameter(argv[ 4], srvm::DEFAULT_INPUT_PARAM_SHOW_CLUSTERS);
		inputShowObjectOnSupport = srvm::getBoolPtrParameter( argv[ 5], srvm::DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT);

		// read the path in which save the file
		centroidLogFilePath = srvm::getPathPtrParameter( argv[ 6], srvm::DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE);

	} else {
		inputPointCloudTopicName = srvm::DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC;
		inputShowOriginalCloud = srvm::DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD;
		inputShowSupportClouds = srvm::DEFAULT_INPUT_PARAM_SHOW_SUPPORTS;
		inputShowClusterClouds = srvm::DEFAULT_INPUT_PARAM_SHOW_CLUSTERS;
		inputShowObjectOnSupport = srvm::DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT;
		centroidLogFilePath = srvm::DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE;
		ROS_WARN_STREAM( "input parameter given to \"" << nodeName << "\" are not correct. Setting all to the default value.");
	}
	/// - log info about this node set up,
	ROS_INFO_STREAM(nodeName << " initialised with:" << endl
					<< "\t show original cloud flag: \t" << srvm::getFlagValueToPrint( inputShowOriginalCloud) << endl
					<< "\t show supports cloud flag: \t" << srvm::getFlagValueToPrint( inputShowSupportClouds) << endl
					<< "\t show clusters cloud flag: \t" << srvm::getFlagValueToPrint(inputShowClusterClouds) << endl
					<< "\t show objects on support flag: \t" << srvm::getFlagValueToPrint( inputShowObjectOnSupport) << endl
					<< "\t input raw cloud topic name: \t\"" << inputPointCloudTopicName << "\"" << endl
					<< "\t raw centroid log file path (empty means do not print): \"" << centroidLogFilePath << "\""
    );

	// eventually (if file path is not "") write raw centroid log header
	pcm::PCManager::writeToFile( "scan id, support idx, cluster idx, centroid X, centroid Y, centroid Z;\n", centroidLogFilePath, true);

	/// - set subscriber to get RGB-D data (from the topic defied by argv[ 1]) that will be processed from #depthAcquisition callback,
    ros::Subscriber subDepth = node.subscribe ( inputPointCloudTopicName, 1, depthAcquisition);

	/// - eventually, create window (with name "Object Table Segmentation") to visualize clouds from this node,
	if(inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport) {
        vis = pcm::PCManager::createVisor("Object Table Segmentation");
        vis->setCameraPosition(-1.88222, 0.632754, 0.534685, -0.650194, 0.490984, 0.6405, -0.081319, 0.036718, 0.99601);
        vis->setCameraFieldOfView(0.8575);
        vis->setCameraClipDistances(0.131668,7.43063);
        vis->setPosition(900,1);
        vis->setSize(960,540);
        vis->addText(log_str_depth, 10, 520, 13, 0.9, 0.9, 0.9, "log_str_depth");
        vis->addText(log_str_supp, 10, 10, 13, 0.9, 0.9, 0.9, "log_str_supp");
        vis_thread = boost::thread( visSpin);
    }

	/// - set the publisher for the node output to contains @code pitt_msgs::msg::ClustersOutput@endcode and named as srvm::TOPIC_OUT_NAME_OBJECT_PERCEPTION (with buffer size 10),
	clusterPub = node.advertise< pitt_msgs::ClustersOutput>( srvm::TOPIC_OUT_NAME_OBJECT_PERCEPTION, 10);

    /// - wait for callbacl and listen for the transformation between the sensor camera (specified through the parameter: srvm::PARAM_NAME_INPUT_CLOUD_REFERENCE_FRAME) and the output frame (specified through the parameter: srvm::PARAM_NAME_OUTPUT_CLOUD_REFERENCE_FRAME)..

	// get the transformation between the kinect optical and the baxter world
	tf::StampedTransform kinectTrans;
    tf::TransformListener listener;
	// initially the transformation is an identity
	pclTransform << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;

	while ( node.ok()){
		try{
			string inputCloudFrame, outputCloudFrame;

			// get ros parameters for cloud frame reference transformation
			node.param<std::string>( srvm::PARAM_NAME_INPUT_CLOUD_REFERENCE_FRAME, inputCloudFrame, srvm::DEFAULT_PARAM_INPUT_CLOUD_REFERENCE_FRAME);
			inputCloudFrame = srvm::getStringParameter( inputCloudFrame, srvm::DEFAULT_PARAM_INPUT_CLOUD_REFERENCE_FRAME); // manage defaults with "."

			node.param<std::string>( srvm::PARAM_NAME_OUTPUT_CLOUD_REFERENCE_FRAME, outputCloudFrame, srvm::DEFAULT_PARAM_OUTPUT_CLOUD_REFERENCE_FRAME);
			outputCloudFrame = srvm::getStringParameter( outputCloudFrame, srvm::DEFAULT_PARAM_OUTPUT_CLOUD_REFERENCE_FRAME); // manage defaults with "."

			// get the transformation between the camera and the world
			listener.waitForTransform(outputCloudFrame, inputCloudFrame, ros::Time(0), ros::Duration( srvm::DEFAULT_TF_WAIT_SECONDS));
			listener.lookupTransform(  outputCloudFrame, inputCloudFrame, ros::Time(0), kinectTrans);

			// retrieve the homogeneous transformation
			pclTransform << kinectTrans.getBasis()[0][0], kinectTrans.getBasis()[0][1], kinectTrans.getBasis()[0][2], kinectTrans.getOrigin().x(),
							kinectTrans.getBasis()[1][0], kinectTrans.getBasis()[1][1], kinectTrans.getBasis()[1][2], kinectTrans.getOrigin().y(),
							kinectTrans.getBasis()[2][0], kinectTrans.getBasis()[2][1], kinectTrans.getBasis()[2][2], kinectTrans.getOrigin().z(),
							0, 							  0, 							0, 							  1;


		} catch ( tf::TransformException &ex){
			ROS_WARN_ONCE( "%s", ex.what());
		}
        ros::spinOnce(); // spin as soon as a new data is available
	}

    if (inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport){
        vis->close();
        vis_thread.join();
    }
	return 0;
}
