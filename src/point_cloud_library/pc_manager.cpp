
#include "../point_cloud_library/pc_manager.h"

namespace pcm {

// ######################################## CONSTANTS ############################################
	// constants for visualizations
	const bool PCManager::DEFAULT_VISUALIZATION_FLAG = false;
	const int PCManager::VISUALIZER_POINT_SIZE = 3;
	const int PCManager::VISUALIZER_POINT_SIZE_BIG = 10;
	const string PCManager::DEFAULT_CLOUD_NAME_SUFFIX = "_cloud";
	const string PCManager::DEFAULT_NORM_NAME_SUFFIX = "_normal";
	const string PCManager::DEFAULT_ORIGINAL_CLOUD_VIEWER_NAME = "original";
	const int PCManager::DEFAULT_NORM_LEVEL = 5;
	const float PCManager::DEFAULT_NORM_SCALE = 0.02f;
	const string PCManager::DEFAULT_VISUALIZER_TITLE = "PointCloud manager";
	// constants for common preprocessing
	const int PCManager::DEFAULT_NORM_SEARCH = 50;//20;
	const float PCManager::DEFAULT_DOWSEAMPLIG_RATE = 0.01f;//0.007f;// 0.0098f;// 0.015f; // TODO: make it as parameter!!!!


// ######################################## GLOBAL VARAIBLES #######################################
	// useful variable to estimate normals
	static NormalEstimation< PointXYZ, Normal> ne;
	static search::KdTree< PointXYZ>::Ptr tree ( new search::KdTree< PointXYZ> ());
	// useful variable for down sampling
	static VoxelGrid< PointXYZ> sor;

// ########################################## STATIC ################################################
	PCLCloudPtr PCManager::copyCloud( PCLCloudPtr input){
		// create cloud
		PointCloud< PointXYZ>::Ptr output ( new PointCloud< PointXYZ>);
		// copy it
		for( int i = 0; i < input->size(); i++){
			// remove Nan
			//if( input->points[ i].x == input->points[ i].x && input->points[ i].y == input->points[ i].y && input->points[ i].z == input->points[ i].z){
				PointXYZ *p ( new PointXYZ( input->points[ i].x, input->points[ i].y, input->points[ i].z));
				output->push_back( *p);
			//}
		}
		return( output);
	}
	PCLNormalPtr PCManager::copyNormals( PCLNormalPtr input){
		PointCloud< Normal>::Ptr output (new PointCloud< Normal>);
		for( int i = 0; i < input->size(); i++)
			output->push_back( input->points[ i]);
		return( output);
	}
	ModelCoefficients::Ptr PCManager::copyCoefficients( ModelCoefficients::Ptr input){
		ModelCoefficients::Ptr output ( new ModelCoefficients);
		for( int i = 0; i < input->values.size(); i++)
			output->values.push_back( input->values[ i]);
		return( output);
	}
	PCLCloudPtr PCManager::downSampling( PCLCloudPtr input){
		return( downSampling( input, DEFAULT_DOWSEAMPLIG_RATE, DEFAULT_DOWSEAMPLIG_RATE, DEFAULT_DOWSEAMPLIG_RATE));
	}
	PCLCloudPtr PCManager::downSampling( PCLCloudPtr input, float span){
		return( downSampling( input, span, span, span));
	}
	PCLCloudPtr PCManager::downSampling( PCLCloudPtr input, float spanX, float spanY, float spanZ){
		PCLCloudPtr output( new PCLCloud);
		sor.setInputCloud( input);
		sor.setLeafSize( spanX, spanY, spanZ); // default 0.008, 0.008, 0.008
		sor.filter( *output);
		return( output);
	}
	PCLNormalPtr PCManager::estimateNormal( PCLCloudPtr input){
		return( estimateNormal( input, DEFAULT_NORM_SEARCH));
	}
	PCLNormalPtr PCManager::estimateNormal( PCLCloudPtr input, int search){
		PCLNormalPtr output( new PCLNormal);
		ne.setSearchMethod( tree);
		ne.setInputCloud( input);
		ne.setKSearch( search);
		ne.compute( *output);
		return( output);
	}
	// to convert ros point cloud ti pcl point cloud and vice-versa
	PointCloud2 PCManager::cloudToRosMsg( PCLCloudPtr input){
		PointCloud2Ptr cl( new PointCloud2);
		toROSMsg( *input, *cl);
		return( *cl);
	}
	PCLCloudPtr PCManager::cloudForRosMsg( PointCloud2Ptr input){
		PCLCloudPtr cl( new PCLCloud);
		fromROSMsg ( *input, *cl);
		return( cl);
	}
	PCLCloudPtr PCManager::cloudForRosMsg( PointCloud2 input){
		PCLCloudPtr cl( new PCLCloud);
		fromROSMsg ( input, *cl);
		return( cl);
	}
	PointCloud2 PCManager::normToRosMsg( PCLNormalPtr input){
		PointCloud2Ptr cl( new PointCloud2);
		toROSMsg( *input, *cl);
		return( *cl);
	}
	PCLNormalPtr PCManager::normForRosMsg( PointCloud2 input){
		PCLNormalPtr cl( new PCLNormal);
		fromROSMsg ( input, *cl);
		return( cl);
	}
	vector< int> PCManager::inlierToVectorMsg( PointIndices::Ptr inliers){
		vector< int> out;
		for( int i = 0; i < inliers->indices.size(); i++)
			if( inliers->indices[ i] != 0)
				out.push_back( inliers->indices[ i]);
		return( out);
	}
	vector< float> PCManager::coefficientToVectorMsg( ModelCoefficients::Ptr coefficients){
		vector< float>* out (new vector< float>);//( coefficients->values.size());
		for( int i = 0; i < coefficients->values.size(); i++)
			out->push_back( coefficients->values[ i]);
		return( *out);
	}

	// returns unix time stamp as a formatted string to append to dialog printing lines
	// Returns the current date and time formatted as %Y-%m-%d_%H.%M.%S
	string PCManager::getFomrattedData(){
		std::time_t t = std::time(NULL);
		char mbstr[20];
		std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H-%M-%S", std::localtime(&t));
		std::string currentDate(mbstr);
		return currentDate;
	}

	//////////////////////////////////////////  VISUALIZATION METHODS
	// method to show visualization window
	PCLVisualizer PCManager::createVisor( string title){
		PCLVisualizer viewer ( new visualization::PCLVisualizer (  title, true));
		viewer->setBackgroundColor( 0, 0, 0);
		viewer->addCoordinateSystem( );
		viewer->initCameraParameters( );
		return viewer;
	}
	// method to show a single point
	void PCManager::updateVisor ( PCLVisualizer viewer, PointXYZ point, int R, int G, int B, string name){
		PCLCloudPtr cloud( new PCLCloud);
		cloud->push_back( point);
		viewer->removePointCloud( name);
		visualization::PointCloudColorHandlerCustom< PointXYZ> color_handler( cloud, R, G, B);
		viewer->addPointCloud< PointXYZ>( cloud, color_handler, name);
		viewer->setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZER_POINT_SIZE_BIG, name);
	}
	// show a single point (with random color))
	void PCManager::updateVisor ( PCLVisualizer viewer, PointXYZ point, string name){
		updateVisor( viewer, point, rand()%255+1, rand()%255+1, rand()%255+1, name);
	}
	// method to show a new cloud
	void PCManager::updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, int R, int G, int B, string name){
		viewer->removePointCloud( name);
		visualization::PointCloudColorHandlerCustom< PointXYZ> color_handler( cloud, R, G, B);
		viewer->addPointCloud< PointXYZ>( cloud, color_handler, name);
		viewer->setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZER_POINT_SIZE, name);
	}

	void PCManager::updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, string name){
		updateVisor( viewer, cloud, rand()%255+1, rand()%255+1, rand()%255+1, name);
	}
	 // method to show a new cloud with normals
	void PCManager::updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, PCLNormalPtr normals, int R, int G, int B, string name){
		viewer->removePointCloud( name);
		viewer->removePointCloud( name + DEFAULT_NORM_NAME_SUFFIX);
		visualization::PointCloudColorHandlerCustom< PointXYZ> color_handler( cloud, R, G, B);
		viewer->addPointCloud< PointXYZ> ( cloud, color_handler, name);
		viewer->setPointCloudRenderingProperties ( visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZER_POINT_SIZE, name);
		viewer->addPointCloudNormals< PointXYZ, Normal> ( cloud, normals, DEFAULT_NORM_LEVEL, DEFAULT_NORM_SCALE, name + DEFAULT_NORM_NAME_SUFFIX);
	}
	// with random color
	void PCManager::updateVisor ( PCLVisualizer viewer, PCLCloudPtr cloud, PCLNormalPtr normals, string name){
		updateVisor( viewer, cloud, normals, rand()%255+1, rand()%255+1, rand()%255+1, name);
	}
	void PCManager::clearVisor( PCLVisualizer viewer){
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
	}

	// get the cloud from indices (to debug data structure for global fit)
	vector< PCLCloudPtr> PCManager::getCloudFromIdx( PCLCloudPtr originalCloud, PrimitiveIdxPtr indices){
		// find maximum map level
		int level = 0;
		for( int i = 0; i < indices->size(); i++)
			if( (* indices)[ i] < level && (* indices)[ i] != -1)
				level = (* indices)[ i];

		// get cloud for each level
		vector< PCLCloudPtr>* outCloud ( new vector< PCLCloudPtr>);
		for( int i = -2; i >= level; i--){ // for each level of the map

			PCLCloudPtr cl ( new PCLCloud);
			for( int j = 0; j < originalCloud->points.size(); j++)
				if( (* indices)[ j] == i)
					cl->push_back( originalCloud->points[ j]);


			outCloud->push_back( cl);
		}
		return( *outCloud);
	}
/*	// to get a cloud from the original one indices map and shape level
	PCLCloudPtr getCloudFromIdx( PCLCloudPtr originalCloud, PrimitiveIdxPtr indices, int level){
		PCLCloudPtr outCloud ( new vector< PCLCloudPtr>);
		PCLCloudPtr cl ( new PCLCloud);
		for( int j = 0; j < originalCloud->points.size(); j++)
			if( (* indices)[ j] == level)
				cl->push_back( originalCloud->points[ j]);
		return( outCloud);
	}*/


// ########################################## PUBLIC ############################################

	bool PCManager::writeToFile( string txt, string filePath, bool append){
		if( ! filePath.empty()){
			ofstream os;
			if( append)
				os.open( filePath.c_str(), std::ios_base::app | std::ios_base::out);
			else os.open( filePath.c_str());
			if ( ! os) {
				os.close();
				ROS_ERROR_ONCE( " !! Error writing to: %s ", filePath.c_str());
				return false;
			} else {
				os << txt.c_str();
				os.close();
				ROS_DEBUG_ONCE( " Data written on path: %s", filePath.c_str());
				return true;
			}
		}
		return false;
	}

	vector< PCLCloudPtr> PCManager::getCloudFromIdx( PrimitiveIdxPtr indices){
		return( getCloudFromIdx( originalCloud, indices));
	}
/*	PCLCloudPtr PCManager::getCloudFromIdx( PrimitiveIdxPtr indices, int level){
		return( getCloudFromIdx( originalCloud, indices, level));
	}*/

	// constructor
	PCManager::PCManager() {
		initialize( DEFAULT_VISUALIZATION_FLAG);
	}
	PCManager::PCManager( bool visualFlag) {
		initialize( visualFlag);
	}

	// deconstructor
	PCManager::~PCManager() {
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}

	// visualize orignal cloud and its norm estimation (if they exist)
	/*void PCManager::visualize(){
		// show original cloud and eventually its normals
		if( originalNorms != NULL)
			updateVisor( visor, originalCloud, originalNorms, 100, 100, 100, DEFAULT_ORIGINAL_CLOUD_VIEWER_NAME);
		else updateVisor( visor, originalCloud, 100, 100, 100, DEFAULT_ORIGINAL_CLOUD_VIEWER_NAME);
		// show primitive w.r.t. visualization flag (does not show normals)
		for( int i = 0; i < primitiveList.size(); i++){
			PCPrimitivePtr shape = primitiveList[ i];
			if( shape->getVisualizationFlag())
				updateVisor( visor, shape->getPrimitiveCloud().makeShared(), rand()%255+1, rand()%255+1, rand()%255+1, shape->getVisualizationName());
		}
	}*/

	////////////////////////////////////////// PRIMITIVE LIST MANAGEMENT
	/*PCPrimitivePtr PCManager::getPrimitiveShape( int idx){
		return( primitiveList[ idx]);
	}
	int PCManager::addPrimitiveShape( string shapeName, PCLCloudPtr cloud, PCLNormalPtr norms, bool visualFlag){
		int idx = primitiveList.size();
		PCPrimitivePtr shape ( new PCPrimitive( shapeName, idx, visualizationFlag, cloud, norms));
		primitiveList.push_back( shape);
		return idx;
	}
	int PCManager::clearPtimitiveShape(){
		int out = primitiveList.size();
		primitiveList.clear();
		return( out);
	}*/


	//////////////////////////////////////////  GETTER METHODS
	PCLCloudPtr PCManager::getOriginalCloud(){
		return( originalCloud);
	}
	PointCloud2 PCManager::getOriginalCloudRosMsg(){
		PointCloud2 out =  cloudToRosMsg( originalCloud);
		return( out);
	}
	PCLNormalPtr PCManager::getOriginalNormal(){ // computed during setOriginalCloud()
		return( originalNorms);
	}
	PointCloud2 PCManager::getOriginalNormalRosMsg(){
		PointCloud2 out = normToRosMsg( originalNorms);
		return( out);
	}
	bool PCManager::getVisualizationFlag(){
		return( visualizationFlag);
	}
	PCLVisualizer PCManager::getVisor(){
		return( visor);
	}


	//////////////////////////////////////////  SETTER METHODS
	void PCManager::setOriginalCloud( PCLCloudPtr cloud){
		setOriginalCloud( cloud, DEFAULT_NORM_SEARCH, DEFAULT_DOWSEAMPLIG_RATE, DEFAULT_DOWSEAMPLIG_RATE, DEFAULT_DOWSEAMPLIG_RATE);
	}
	void PCManager::setOriginalCloud( PCLCloudPtr cloud, int normSearch, float downSpanX, float downSpanY, float downSpanZ){
		PCLCloudPtr cloudSampled = downSampling( cloud, downSpanX, downSpanY, downSpanZ);
		PCLNormalPtr normals = estimateNormal( cloudSampled, normSearch);
		originalCloud->clear();
		originalNorms->clear();
		originalCloud = cloudSampled;
		originalNorms = normals;
	}
	void PCManager::setOriginalCloud( PointCloud2Ptr cloud){
		setOriginalCloud( cloud, DEFAULT_NORM_SEARCH, DEFAULT_DOWSEAMPLIG_RATE, DEFAULT_DOWSEAMPLIG_RATE, DEFAULT_DOWSEAMPLIG_RATE);
	}
	void PCManager::setOriginalCloud( PointCloud2Ptr cloud, int normSearch, float downSpanX, float downSpanY, float downSpanZ){
		PCLCloudPtr cl = cloudForRosMsg( *cloud);
		setOriginalCloud( cl, normSearch, downSpanX, downSpanY, downSpanZ);
	}
	void PCManager::setVisualizationFlag( bool flag){
		if( flag){
			if( !visualizationFlag){
				visor = createVisor( DEFAULT_VISUALIZER_TITLE);
			}
		} else {
			if( visualizationFlag){
				visor->close();
			}
		}
		visualizationFlag = flag;
	}


// ########################################## PRIVATE ##########################################

	// comon initialization for all constructors
	void PCManager::initialize( bool flag){
		// set visualization
		visualizationFlag = flag;
		if( flag)
			visor = createVisor( DEFAULT_VISUALIZER_TITLE);

		// initialize original cloud
		//originalCloud = new PCLCloud;
	}


} /* namespace pcm */
