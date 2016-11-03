/**
 * Project@build<br>
 * /Project@build/[Source directory]/primitive_identification_tagging_and_tracking/pitt_object_table_segmentation/src/point_cloud_library/srv-manager.h<br>
 * <p>
 * Created on: Mar 7, 2016
 *     Author: luca-phd
 * Institution: University of Genoa, DIBRIS, EmaroLab
 * </p>
 * <p>
 * ...
 * </p>
 */

#ifndef SRV_MANAGER_H_
#define SRV_MANAGER_H_

#include <std_msgs/String.h>
#include "../point_cloud_library/pc_manager.h"

using namespace std;

	namespace srvm {

        // name of the services
        const string SRV_NAME_DEEP_FILTER = "deep_filter_srv";
        const string SRV_NAME_SUPPORT_FILTER = "support_segmentation_srv";
        const string SRV_NAME_CUSTER_FILTER = "cluster_Segmentation_srv";
        const string SRV_NAME_ARM_FILTER = "arm_filter_srv";
        const string SRV_NAME_RANSAC_SPHERE_FILTER = "sphere_segmentation_srv";
        const string SRV_NAME_RANSAC_CYLINDER_FILTER = "cylinder_segmentation_srv";
        const string SRV_NAME_RANSAC_CONE_FILTER = "cone_segmentation_srv";
        const string SRV_NAME_RANSAC_PLANE_FILTER = "plane_segmentation_srv";

        // ROS parameters names
        const string PARAM_NAME_INPUT_CLOUD_REFERENCE_FRAME = "/pitt/ref_frame/input_cloud";
        const string PARAM_NAME_OUTPUT_CLOUD_REFERENCE_FRAME = "/pitt/ref_frame/output_cloud";
        const string PARAM_NAME_DEEP_SRV_Z_THRESHOLD = "/pitt/service/deep_filter/z_threshold";

        const string PARAM_NAME_ARM_SRV_MIN_FOREARM_BOX  = "/pitt/srv/arm_filter/min_forearm_box";
        const string PARAM_NAME_ARM_SRV_MAX_FOREARM_BOX  = "/pitt/srv/arm_filter/max_forearm_box";
        const string PARAM_NAME_ARM_SRV_MIN_ELBOW_BOX  = "/pitt/srv/arm_filter/min_elbow_box";
        const string PARAM_NAME_ARM_SRV_MAX_ELBOW_BOX  = "/pitt/srv/arm_filter/max_elbow_box";

        const string PARAM_NAME_CLUSTER_TOLERANCE = "/pitt/srv/cluster_segmentation/tolerance";
        const string PARAM_NAME_CLUSTER_MIN_RATE = "/pitt/srv/cluster_segmentation/min_rate";
        const string PARAM_NAME_CLUSTER_MAX_RATE = "/pitt/srv/cluster_segmentation/max_rate";
        const string PARAM_NAME_CLUSTER_MIN_INPUT_SIZE = "/pitt/srv/cluster_segmentation/min_input_size";

        const string PARAM_NAME_SPHERE_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/sphere_segmentation/normal_distance_weight";
        const string PARAM_NAME_SPHERE_DISTANCE_TH = "/pitt/srv/sphere_segmentation/distance_th";
        const string PARAM_NAME_SPHERE_MAX_ITERATION_LIMIT = "/pitt/srv/sphere_segmentation/max_iter_limit";
        const string PARAM_NAME_SPHERE_MIN_RADIUS_LIMIT =  "/pitt/srv/sphere_segmentation/min_radius_limit";
        const string PARAM_NAME_SPHERE_MAX_RADIUS_LIMIT = "/pitt/srv/sphere_segmentation/max_radius_limit";
        const string PARAM_NAME_SPHERE_EPS_ANGLE_TH = "/pitt/srv/sphere_segmentation/eps_angle_th";
        const string PARAM_NAME_SPHERE_MIN_OPENING_ANGLE_DEGREE = "/pitt/srv/sphere_segmentation/min_opening_angle_deg";
        const string PARAM_NAME_SPHERE_MAX_OPENING_ANGLE_DEGREE = "/pitt/srv/sphere_segmentation/max_opening_angle_deg";
        const string PARAM_NAME_SPHERE_MIN_INLIERS = "/pitt/srv/sphere_segmentation/min_inliers";

        const string PARAM_NAME_CYLINDER_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/cylinder_segmentation/normal_distance_weight";
        const string PARAM_NAME_CYLINDER_DISTANCE_TH = "/pitt/srv/cylinder_segmentation/distance_th";
        const string PARAM_NAME_CYLINDER_MAX_ITERATION_LIMIT = "/pitt/srv/cylinder_segmentation/max_iter_limit";
        const string PARAM_NAME_CYLINDER_MIN_RADIUS_LIMIT =  "/pitt/srv/cylinder_segmentation/min_radius_limit";
        const string PARAM_NAME_CYLINDER_MAX_RADIUS_LIMIT = "/pitt/srv/cylinder_segmentation/max_radius_limit";
        const string PARAM_NAME_CYLINDER_EPS_ANGLE_TH = "/pitt/srv/cylinder_segmentation/eps_angle_th";
        const string PARAM_NAME_CYLINDER_MIN_OPENING_ANGLE_DEGREE = "/pitt/srv/cylinder_segmentation/min_opening_angle_deg";
        const string PARAM_NAME_CYLINDER_MAX_OPENING_ANGLE_DEGREE = "/pitt/srv/cylinder_segmentation/max_opening_angle_deg";
        const string PARAM_NAME_CYLINDER_MIN_INLIERS = "/pitt/srv/cylinder_segmentation/min_inliers";

        const string PARAM_NAME_CONE_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/cone_segmentation/normal_distance_weight";
        const string PARAM_NAME_CONE_DISTANCE_TH = "/pitt/srv/cone_segmentation/distance_th";
        const string PARAM_NAME_CONE_MAX_ITERATION_LIMIT = "/pitt/srv/cone_segmentation/max_iter_limit";
        const string PARAM_NAME_CONE_MIN_RADIUS_LIMIT =  "/pitt/srv/cone_segmentation/min_radius_limit";
        const string PARAM_NAME_CONE_MAX_RADIUS_LIMIT = "/pitt/srv/cone_segmentation/max_radius_limit";
        const string PARAM_NAME_CONE_EPS_ANGLE_TH = "/pitt/srv/cone_segmentation/eps_angle_th";
        const string PARAM_NAME_CONE_MIN_OPENING_ANGLE_DEGREE = "/pitt/srv/cone_segmentation/min_opening_angle_deg";
        const string PARAM_NAME_CONE_MAX_OPENING_ANGLE_DEGREE = "/pitt/srv/cone_segmentation/max_opening_angle_deg";
        const string PARAM_NAME_CONE_MIN_INLIERS = "/pitt/srv/cone_segmentation/min_inliers";

        const string PARAM_NAME_PLANE_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/plane_segmentation/normal_distance_weight";
        const string PARAM_NAME_PLANE_DISTANCE_TH = "/pitt/srv/plane_segmentation/distance_th";
        const string PARAM_NAME_PLANE_MAX_ITERATION_LIMIT = "/pitt/srv/plane_segmentation/max_iter_limit";
        const string PARAM_NAME_PLANE_EPS_ANGLE_TH = "/pitt/srv/plane_segmentation/eps_angle_th";
        const string PARAM_NAME_PLANE_MIN_OPENING_ANGLE_DEGREE = "/pitt/srv/plane_segmentation/min_opening_angle_deg";
        const string PARAM_NAME_PLANE_MAX_OPENING_ANGLE_DEGREE = "/pitt/srv/plane_segmentation/max_opening_angle_deg";
        const string PARAM_NAME_PLANE_MIN_INLIERS = "/pitt/srv/plane_segmentation/min_inliers";

        // supports_segmentation
        const string PARAM_NAME_MIN_ITERATIVE_CLOUD_PERCENTAGE = "/pitt/srv/supports_segmentation/min_iter_cloud_percent";
        const string PARAM_NAME_MIN_ITERATIVE_SUPPORT_PERCENTAGE = "/pitt/srv/supports_segmentation/min_iter_support_percent";
        const string PARAM_NAME_HORIZONTAL_VARIANCE_THRESHOLD = "/pitt/srv/supports_segmentation/horizontal_variance_th";
        const string PARAM_NAME_RANSAC_IN_SHAPE_DISTANCE_POINT_THRESHOLD = "/pitt/srv/supports_segmentation/in_shape_distance_th";
        const string PARAM_NAME_RANSAC_MODEL_NORMAL_DISTANCE_WEIGHT = "/pitt/srv/supports_segmentation/normal_distance_weight";
        const string PARAM_NAME_RANSAC_MAX_ITERATION_THRESHOLD = "/pitt/srv/supports_segmentation/max_iter";
        const string PARAM_NAME_HORIZONTAL_AXIS = "/pitt/srv/supports_segmentation/horizontal_axis";
        const string PARAM_NAME_SUPPORT_EDGE_REMOVE_OFFSET = "/pitt/srv/supports_segmentation/edge_remove_offset";

        // parameter (not ros) default value
        const string DEFAULT_PARAM_INPUT_CLOUD_REFERENCE_FRAME = "/camera_depth_optical_frame";
        const string DEFAULT_PARAM_OUTPUT_CLOUD_REFERENCE_FRAME = "/world";
        /// the name of the input topic
        const string DEFAULT_INPUT_PARAM_RAW_CLOUD_TOPIC = "/camera/depth/points"; 	// default for freenect driver
        const string DEFAULT_INPUT_PARAM_CENTROID_LOG_FILE = ""; 					// empty do not print
        const bool DEFAULT_INPUT_PARAM_SHOW_ORIGINAL_CLOUD = false; 				// with norms [white]
        const bool DEFAULT_INPUT_PARAM_SHOW_SUPPORTS = false;  						// [brown]
        const bool DEFAULT_INPUT_PARAM_SHOW_OBJECT_ON_SUPPORT = false; 				// as a unique cloud [orange]
        const bool DEFAULT_INPUT_PARAM_SHOW_CLUSTERS = false;						// as separate clusters [with random colors]
        const string DEFAULT_PARAM_ARM_SRV_CAMERA_FRAME = "/camera_depth_optical_frame";
        const string DEFAULT_PARAM_ARM_SRV_RIGHT_FOREARM_FRAME = "/right_lower_forearm";
        const string DEFAULT_PARAM_ARM_SRV_LEFT_FOREARM_FRAME = "/left_lower_forearm";
        const string DEFAULT_PARAM_ARM_SRV_RIGHT_ELBOW_FRAME = "/right_lower_elbow";
        const string DEFAULT_PARAM_ARM_SRV_LEFT_ELBOW_FRAME = "/left_lower_elbow";
        const bool DEFAULT_PARAM_ARM_SRV_SHOW_CLOUDS = false;	// [red filtered points (arm)] [green remaining points]

        // topics (between nodes) names
        // TODO: ADJUST NAMES
        const string TOPIC_OUT_NAME_OBJECT_PERCEPTION = "obj_segmentation/ClusterOutput";

        // value to set service parameter as default
        const int DEFAULT_SERVICE_PARAMETER_REQUEST = -1;
        const float DEFAULT_SERVICE_PARAMETER_REQUEST_F = -1.0f;
        const float DEFAULT_SERVICE_ARRAY_PARAMETER_REQUEST[1] = {-1};
        const vector<float> DEFAULT_SERVICE_VEC_PARAMETER_REQUEST(DEFAULT_SERVICE_ARRAY_PARAMETER_REQUEST,
                                                                  DEFAULT_SERVICE_ARRAY_PARAMETER_REQUEST +
                                                                  sizeof(DEFAULT_SERVICE_ARRAY_PARAMETER_REQUEST)
                                                                  /sizeof(float));
        const float DEFAULT_TF_WAIT_SECONDS = 2.0f;

        const string DEFAULT_SYMBOL = ".";

        string getStringParameter( string input, const string defaultValue){
            if( input == DEFAULT_SYMBOL)
                return defaultValue;
            return input;
        }
        string getStringPtrParameter( char *input, const string defaultValue){
            string inputStr( input);
            return getStringParameter( inputStr, defaultValue);
        }

        bool getBoolParameter( string input, const bool defaultValue){
            if( input == DEFAULT_SYMBOL)
                return defaultValue;
            return (bool) strtol( input.c_str(), NULL, 0);
        }
        bool getBoolPtrParameter( char *input, const bool defaultValue){
            string inputStr( input);
            return getBoolParameter( inputStr, defaultValue);
        }

        string getPathParameter( string input, const string defaultValue){
            if( input == DEFAULT_SYMBOL)
                return defaultValue;
            if( input.find( "..") != std::string::npos){
                string out = input.substr(0, input.size() - 2); // remove the ".."
                return out + pcm::PCManager::getFomrattedData();
            }
            return input;
        }
        string getPathPtrParameter( char *input, const string defaultValue){
            string inputStr( input);
            return getPathParameter( inputStr, defaultValue);
        }

        float getServiceFloatParameter( float input, const float defaultValue){
            if( input >= 0.0f)
                return input;
            return defaultValue;
        }
        int getServiceIntParameter( int input, const int defaultValue){
            if( input >= 0)
                return input;
            return defaultValue;
        }
        string getServiceStringParameter( string input, const string defaultValue){
            return getStringParameter( input, defaultValue);
        }
        vector<float> getService3DArrayParameter( vector<float> input, const vector<float> defaultValue){
            if( input.size() == 3)
                return input;
            return defaultValue;
        }
        vector< float> get3DArray( const float values[]){
            vector<float> vec( values, values + 3);
            return vec;
        }
        vector<float> getService3DArrayParameter( vector<float> input, const float defaultValue[]){
            vector<float> defaultVector = get3DArray( defaultValue);
            return getService3DArrayParameter( input, defaultVector);
        }


        string getFlagValueToPrint( bool flag){
            if( flag)
                return  "true  (1)";
            else return "false (0)";
        }
        string getArrayToPrint( vector< float> arr){
            string out = "[";
            if( arr.size() == 0)
                return out + "]";
            for( int i = 0; i < arr.size(); i++)
                if( i < arr.size() - 1)
                    out += boost::to_string( arr[ i]) + ", ";
                else out += boost::to_string( arr[ i]) + "]";
            return out;
        }
} /* namespace srvm */



#endif /* SRV_MANAGER_H_ */
