#include <pcl_ros/point_cloud.h> // defnes structure sensor_msgs:pointCloud2
#include <std_msgs/Float64.h>	 // for kinect angle

#include <pcl/common/transforms.h> // to transform point cloud reference frame

#include <eigen3/Eigen/Dense>	 // for ground axes estimation based on baxter orientation
#include <eigen3/Eigen/Core>
#include <math.h>
#include <tf/transform_listener.h> // to get the kinect frame
#include <tf/tf.h>

#include "pitt_msgs/DeepFilter.h"
#include "pitt_msgs/SupportSegmentation.h"
#include "pitt_msgs/ClusterSegmentation.h"
#include "pitt_msgs/ArmFilter.h"
// for my messages (.msg files)
#include "pitt_msgs/Support.h"
#include "pitt_msgs/InliersCluster.h"
#include "pitt_msgs/ClustersOutput.h"

#include "point_cloud_library/pc_manager.h" // for my static library
#include "point_cloud_library/srv_manager.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>

//using namespace pitt_object_table_segmentation;
using namespace pitt_msgs;
using namespace ros;
using namespace sensor_msgs;
using namespace pcm;
using namespace srvm;
using namespace tf;

ros::NodeHandle* nh_ptr = NULL;

typedef boost::shared_ptr< vector< Support> > InlierSupportsPtr;
typedef vector< Support> InlierSupports;
typedef boost::shared_ptr< vector< InliersCluster> > InlierClusterPtr;
typedef vector< InliersCluster> InlierClusters;

// global input parameter used during node initialization
bool inputShowSupportClouds, inputShowOriginalCloud, inputShowClusterClouds, inputShowObjectOnSupport;
string centroidLogFilePath;
// input parameters used in service (populate on the main node spin)

string log_str_depth = "Loading...";
string log_str_supp = "Loading...";

// used only for logging
long scanId = 0;

// the cloud is not processed if has less number of points
static const int MIN_POINT_IN_ORIGINAL_CLOUD = 30;

// global private variable
pcm::PCManager* manager = new pcm::PCManager( false); // true (visualize)
boost::shared_ptr< visualization::PCLVisualizer> vis; // to visualize cloud
Publisher clusterPub; // variable to publish the output on the call back
boost::thread vis_thread;
boost::mutex vis_mutex;

void visSpin(){
    while(!vis->wasStopped()){
        boost::mutex::scoped_lock updateLock(vis_mutex);
        vis->spinOnce(100);
    }
}

// call deep filter service (modifies the input data)
// remove all the point over a threshold on the z-axis of the camera frame
/*!
 * AAAAA
 * \brief BBBB
 * @param cloud
 * @return
 */
bool callDeepFilter( PCLCloudPtr& cloud){

	float inputDepthThreshold;

	nh_ptr->param(srvm::PARAM_NAME_DEEP_SRV_Z_THRESHOLD, inputDepthThreshold, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);

	if (inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport) {
        boost::mutex::scoped_lock updateLock(vis_mutex);
		log_str_depth = boost::str(boost::format("DEPTH THRESHOLD: %f  (-1 -> default value)")
                                   % inputDepthThreshold);
		vis->updateText(log_str_depth, 10, 520, "log_str_depth");
	}

	// initialise deep filter server caller
	DeepFilter srvDeep;
	ServiceClient clientDeep = nh_ptr->serviceClient< DeepFilter>( srvm::SRV_NAME_DEEP_FILTER);

	// set input data and parameters
	srvDeep.request.input_cloud = PCManager::cloudToRosMsg( cloud);
	srvDeep.request.deep_threshold = inputDepthThreshold;

	// call service
	if( clientDeep.call( srvDeep)){ // get the repose
		cloud = PCManager::cloudForRosMsg( srvDeep.response.cloud_closer);
		// far = srv.response.cloud_further; // not used
		return true;
	} else { // error on getting repose
		ROS_ERROR_STREAM( " error on calling service " << clientDeep.getService());
		return false;
	}
}

//filter points belonging to the robot arms
bool callArmFilter( PCLCloudPtr& cloud){

	vector< float> forearmMinBox, forearmMaxBox, elbowMinBox, elbowMaxBox;

	// initialise deep filter server caller
	ServiceClient clientArm = nh_ptr->serviceClient< ArmFilter>( srvm::SRV_NAME_ARM_FILTER);
	ArmFilter armFilterSrv;

	// arm filtering service
    float arr[1] = {-1};
	vector<float> vec(arr, arr +sizeof(arr)/sizeof(float));
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MIN_FOREARM_BOX, forearmMinBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MAX_FOREARM_BOX, forearmMaxBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MIN_ELBOW_BOX, elbowMinBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
	nh_ptr->param( srvm::PARAM_NAME_ARM_SRV_MAX_ELBOW_BOX, elbowMaxBox, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);

	// set input data and parameters
	armFilterSrv.request.input_cloud = PCManager::cloudToRosMsg( cloud);
	armFilterSrv.request.forearm_bounding_box_min_value = forearmMinBox;
	armFilterSrv.request.forearm_bounding_box_max_value = forearmMaxBox;
	armFilterSrv.request.elbow_bounding_box_min_value = elbowMinBox;
	armFilterSrv.request.elbow_bounding_box_max_value = elbowMaxBox;

	// call service
	if( clientArm.call( armFilterSrv)){ // get the repose
		cloud = PCManager::cloudForRosMsg( armFilterSrv.response.armless_cloud);
		return true;
    } else { // error on getting repose
    	ROS_ERROR_STREAM( " error on calling service " << clientArm.getService());
        return false;
    }
}

// call support segmentation server which returns all the points and inliers belongs to different horizontal plane (w.r.t. z axis)
// by default, since the cloud is described w.r.t. baxter world, the perpendicular to the ground floar is -z(wold)
InlierSupportsPtr  callSupportFilter( PCLCloudPtr inputCloud, PCLNormalPtr normal){

	// call support service
	ServiceClient client = nh_ptr->serviceClient< SupportSegmentation>( srvm::SRV_NAME_SUPPORT_FILTER);
	SupportSegmentation srv;

	// set input data
	srv.request.input_cloud = PCManager::cloudToRosMsg( inputCloud);
	srv.request.input_norm = PCManager::normToRosMsg( normal);

	// get input parameter or set default (default value: -1)
    vector<float> horizontalAxis, edgeRemoveOffset;

    nh_ptr->param(srvm::PARAM_NAME_MIN_ITERATIVE_CLOUD_PERCENTAGE,
                  srv.request.min_iterative_cloud_percentual_size, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param(srvm::PARAM_NAME_MIN_ITERATIVE_SUPPORT_PERCENTAGE,
                  srv.request.min_iterative_plane_percentual_size, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param(srvm::PARAM_NAME_HORIZONTAL_VARIANCE_THRESHOLD,
                  srv.request.variance_threshold_for_horizontal, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param(srvm::PARAM_NAME_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD,
                  srv.request.ransac_distance_point_in_shape_threshold, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param(srvm::PARAM_NAME_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT,
                  srv.request.ransac_model_normal_distance_weigth, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST_F);
    nh_ptr->param(srvm::PARAM_NAME_RANSAC_MAX_ITERATION_THRESHOLD,
                  srv.request.ransac_max_iteration_threshold, srvm::DEFAULT_SERVICE_PARAMETER_REQUEST);
    nh_ptr->param(srvm::PARAM_NAME_HORIZONTAL_AXIS,
                  horizontalAxis, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);
    nh_ptr->param(srvm::PARAM_NAME_SUPPORT_EDGE_REMOVE_OFFSET,
                  edgeRemoveOffset, srvm::DEFAULT_SERVICE_VEC_PARAMETER_REQUEST);

    srv.request.horizontal_axis = horizontalAxis;
    srv.request.support_edge_remove_offset = edgeRemoveOffset;
	// call service
	InlierSupportsPtr objs ( new InlierSupports( srv.response.supports_description.size()));
	if( client.call( srv))
		*objs = srv.response.supports_description; // get the response
	else
		ROS_ERROR_STREAM( " error on calling service " << client.getService());

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

// clusterize objects over the input support cloud
InlierClusterPtr callClusterSegmentation( PCLCloudPtr cloud){
	// call cluster service
	ServiceClient client = nh_ptr->serviceClient< ClusterSegmentation>( srvm::SRV_NAME_CUSTER_FILTER);
	ClusterSegmentation srv;

	// set input data
	srv.request.cloud = PCManager::cloudToRosMsg( cloud);

	// call the service
	InlierClusterPtr inc ( new InlierClusters( srv.response.cluster_objs.size()));
	if( client.call( srv))// get the response
		*inc = srv.response.cluster_objs;
	else
		ROS_ERROR_STREAM( " error on calling service " << client.getService());
	return( inc);
}

// callback on Kinect depth published data
Eigen::Matrix4f pclTransform;
//static string centroidFileLog;
void depthAcquisition( const PointCloud2Ptr& input){

	string centroidFileLog = "";

	// get kinect inputs as a standard pcl cloud
	PCLCloudPtr rawCloud = PCManager::cloudForRosMsg( input);

	// compute down-sampling
	PCLCloudPtr cloud = PCManager::downSampling( rawCloud); //using default DOWN_SAMPLING_RATE

	// apply deep filter server
	if( callDeepFilter( cloud)){

		// filter out robot arms
		if( callArmFilter( cloud)){

			// transform point cloud to world frame
			PCLCloudPtr worldCloud( new PCLCloud);
			pcl::transformPointCloud( *cloud, *worldCloud, pclTransform);

			// skip if too few input point (avoid error on compute normals)
			if( worldCloud->points.size() > MIN_POINT_IN_ORIGINAL_CLOUD){
				// compute normal
				PCLNormalPtr normal = PCManager::estimateNormal( worldCloud); // using default ESTIMATE_NORMAL_SPAN
				// show original cloud as gray points
				if( inputShowOriginalCloud) {
                    boost::mutex::scoped_lock lock(vis_mutex);
                    PCManager::updateVisor(vis, worldCloud, normal, 220, 220, 220, "original");
                }

				// compute supports
				InlierSupportsPtr supports = callSupportFilter( worldCloud, normal);

				if( supports->size() > 0){ // at least one support
					for( int i = 0; i < supports->size(); i++){ // for all the found supports
						if( inputShowSupportClouds){
							// get horizontal plane from service response
							PCLCloudPtr support = PCManager::cloudForRosMsg( (* supports)[ i].support_cloud);
							// show points with brown colors
                            boost::mutex::scoped_lock lock(vis_mutex);
							PCManager::updateVisor( vis, support, 102, 55, 55,  "table" +
                                    boost::lexical_cast<std::string>( i));
						}
						// show object on the horizontal plane
						PCLCloudPtr onSupport = PCManager::cloudForRosMsg( (* supports)[ i].on_support_cloud);
						// show points
						if( inputShowObjectOnSupport) {
                            boost::mutex::scoped_lock lock(vis_mutex);
                            PCManager::updateVisor(vis, onSupport, 255, 183, 131, "object" +
                                    boost::lexical_cast<std::string>(i));
                        }

						// compute clusters
						InlierClusterPtr clusters = callClusterSegmentation( onSupport);

						// prepare node output
						boost::shared_ptr< ClustersOutput> out ( new ClustersOutput);
						if( clusters->size() > 0){ // at least one cluster
							for( int j = 0; j < clusters->size(); j++){ // for all the clusters
								InliersCluster clusterObject = (* clusters)[ j];
								// append this cluster to output
								out->cluster_objs.push_back( clusterObject);

								// get cluster
								PCLCloudPtr clusterCloud = PCManager::cloudForRosMsg(clusterObject.cloud);

								if(inputShowClusterClouds) { // visualize cluster
                                    boost::mutex::scoped_lock lock(vis_mutex);
                                    PCManager::updateVisor(vis, clusterCloud, "clusterPlane" +
                                                                              boost::lexical_cast<std::string>(j));
                                }

								// prepare detached cluster center of mass logs
								centroidFileLog += boost::lexical_cast<std::string>(scanId) + ", " +
                                        boost::lexical_cast<std::string>( i) + ", " +
                                        boost::lexical_cast<std::string>( j) + ", " +
                                        boost::lexical_cast<std::string>( clusterObject.x_centroid) + ", " +
                                        boost::lexical_cast<std::string>( clusterObject.y_centroid) + ", " +
                                        boost::lexical_cast<std::string>( clusterObject.z_centroid) + ";\n";
							}
							// publish the center of mass of the detached cluster for a specific support
							clusterPub.publish( out);
						}
					}
				}
			}
		}
	}
	// print on screen
	ROS_INFO_STREAM( "raw clusters data: [scan id, support idx, cluster idx, centroid X, cenntroid Y, centroid Z;\\n]" << endl << centroidFileLog);
	// eventually print on file
	PCManager::writeToFile( centroidFileLog, centroidLogFilePath, true);
	scanId += 1;
}


/**
 * This method implements the main node loop and it spins as soon as a new data is available
 * in the input topic. Particularly, the input topic can be specified through its name into the parameter .....
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv){
	// Instantiate the node
	string nodeName = "obj_segmentation";
	int numberOfInputParameter = 6;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle node;
    nh_ptr = &node;

	// read input parameters (the one that are set only on node start up)
	std::string inputPointCloudTopicName;

    if( argc == numberOfInputParameter + 1){
		// args[ 0] is the path to the executable file

		// read the name of the input cloud topic
		inputPointCloudTopicName = srvm::getStringPtrParameter( argv[ 1], DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC);

		// read the flags to show point cloud plots
		inputShowOriginalCloud = srvm::getBoolPtrParameter( argv[ 2], DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD);
		inputShowSupportClouds = srvm::getBoolPtrParameter( argv[ 3], DEFAULT_INPUT_PARAM_SHOW_SUPPORTS);
		inputShowClusterClouds = srvm::getBoolPtrParameter(argv[ 4], DEFAULT_INPUT_PARAM_SHOW_CLUSTERS);
		inputShowObjectOnSupport = srvm::getBoolPtrParameter( argv[ 5], DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT);

		// read the path in which save the file
		centroidLogFilePath = srvm::getPathPtrParameter( argv[ 6], DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE);

	} else {
		inputPointCloudTopicName = DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC;
		inputShowOriginalCloud = DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD;
		inputShowSupportClouds = DEFAULT_INPUT_PARAM_SHOW_SUPPORTS;
		inputShowClusterClouds = DEFAULT_INPUT_PARAM_SHOW_CLUSTERS;
		inputShowObjectOnSupport = DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT;
		centroidLogFilePath = DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE;
		ROS_WARN_STREAM( "input parameter given to \"" << nodeName << "\" are not correct. Setting all to the default value.");
	}
	// log the value coming from using inputs
	ROS_INFO_STREAM(nodeName << " initialised with:" << endl
					<< "\t show original cloud flag: \t" << getFlagValueToPrint( inputShowOriginalCloud) << endl
					<< "\t show supports cloud flag: \t" << getFlagValueToPrint( inputShowSupportClouds) << endl
					<< "\t show clusters cloud flag: \t" << getFlagValueToPrint(inputShowClusterClouds) << endl
					<< "\t show objects on support flag: \t" << getFlagValueToPrint( inputShowObjectOnSupport) << endl
					<< "\t input raw cloud topic name: \t\"" << inputPointCloudTopicName << "\"" << endl
					<< "\t raw centroid log file path (empty means do not print): \"" << centroidLogFilePath << "\"");

	// eventually (if file path is not "") write raw centroid log header
	PCManager::writeToFile( "scan id, support idx, cluster idx, centroid X, centroid Y, centroid Z;\n", centroidLogFilePath, true);

	// set subscriber to get kinect depth points given from input parameter
	Subscriber subDepth = node.subscribe ( inputPointCloudTopicName, 1, depthAcquisition);

	// create window to visualize clouds
	if(inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport) {
        vis = PCManager::createVisor("Object Table Segmentation");
        vis->setCameraPosition(-1.88222, 0.632754, 0.534685, -0.650194, 0.490984, 0.6405, -0.081319, 0.036718, 0.99601);
        vis->setCameraFieldOfView(0.8575);
        vis->setCameraClipDistances(0.131668,7.43063);
        vis->setPosition(900,1);
        vis->setSize(960,540);
        vis->addText(log_str_depth, 10, 520, 13, 0.9, 0.9, 0.9, "log_str_depth");
        vis->addText(log_str_supp, 10, 10, 13, 0.9, 0.9, 0.9, "log_str_supp");
        vis_thread = boost::thread(visSpin);
    }

	// set publisher for cluster out
	clusterPub = node.advertise< ClustersOutput>( srvm::TOPIC_OUT_NAME_OBJECT_PERCEPTION, 10);

	// get the transformation between the kinect optical and the baxter world
	StampedTransform kinectTrans;
	TransformListener listener;
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
			listener.waitForTransform(outputCloudFrame, inputCloudFrame, Time(0), Duration( srvm::DEFAULT_TF_WAIT_SECONDS));
			listener.lookupTransform(  outputCloudFrame, inputCloudFrame, Time(0), kinectTrans);

			// retrieve the homogeneous transformation
			pclTransform << kinectTrans.getBasis()[0][0], kinectTrans.getBasis()[0][1], kinectTrans.getBasis()[0][2], kinectTrans.getOrigin().x(),
							kinectTrans.getBasis()[1][0], kinectTrans.getBasis()[1][1], kinectTrans.getBasis()[1][2], kinectTrans.getOrigin().y(),
							kinectTrans.getBasis()[2][0], kinectTrans.getBasis()[2][1], kinectTrans.getBasis()[2][2], kinectTrans.getOrigin().z(),
							0, 							  0, 							0, 							  1;


		} catch ( TransformException &ex){
			ROS_WARN_ONCE( "%s", ex.what());
		}
		spinOnce(); // spin as soon as a new data is available
	}

    if (inputShowOriginalCloud || inputShowSupportClouds || inputShowClusterClouds || inputShowObjectOnSupport){
        vis->close();
        vis_thread.join();
    }
	return 0;
}
