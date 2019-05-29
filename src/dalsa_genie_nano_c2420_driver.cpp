//This code is based on the GigE Vision Library GenICam C++ Example Program by DALSA, 2015

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "gevapi.h"	
#include "GenApi/GenApi.h"
#include "dalsa_genie_nano_c2420/SapX11Util.h"
#include "dalsa_genie_nano_c2420/X_Display_utils.h"
#include "dalsa_genie_nano_c2420/dalsa_genie_nano_c2420_utils.h"

#undef index //removing this macro allows OpenCV to compile
//#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "dalsa_genie_nano_c2420_driver");
  ros::NodeHandle n_p("~");
  ros::NodeHandle n;

  std::string dalsa_camera_frame;
  n_p.param<std::string>("dalsa_camera_frame", dalsa_camera_frame, "dalsa_link");
  std::string dalsa_camera_topic;
  n_p.param<std::string>("dalsa_camera_topic", dalsa_camera_topic, "dalsa_camera");

  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>(dalsa_camera_topic, 100);
  ros::Publisher img_pub_mono;
  cv_bridge::CvImagePtr cv_ptr;

  int camIndex;
  n_p.param("camera_index", camIndex, 0); //only used when more than 1 camera is connected 
  bool publish_mono;
  n_p.param("publish_mono", publish_mono, false);	//also publish monochromatic image

  if (publish_mono){
  	std::string dalsa_camera_mono_topic;
    n_p.param<std::string>("dalsa_camera_mono_topic", dalsa_camera_mono_topic, "dalsa_camera_mono");
  	img_pub_mono = n.advertise<sensor_msgs::Image>(dalsa_camera_mono_topic, 100);
  }

  bool use_synchronous_buffer_cycling;
  n_p.param("use_synchronous_buffer_cycling", use_synchronous_buffer_cycling, false);

  bool tune_streaming_threads;
  n_p.param("tune_streaming_threads", tune_streaming_threads, false);

  bool turbo_mode;
  n_p.param("turbo_mode", turbo_mode, false);


  // Print Params:
  // ROS_INFO("dalsa_camera_frame: %s", dalsa_camera_frame.c_str());
  // ROS_INFO("camera_index: %d", camIndex);
  // ROS_INFO("publish_mono: %s", publish_mono ? "true" : "false");
  // ROS_INFO("use_synchronous_buffer_cycling: %s", use_synchronous_buffer_cycling ? "true" : "false");
  // ROS_INFO("tune_streaming_threads: %s", tune_streaming_threads ? "true" : "false");
  // ROS_INFO("turbo_mode: %s", turbo_mode ? "true" : "false");

  //Setting up camera capture:
  void *m_latestBuffer = NULL;
  GEV_DEVICE_INTERFACE  pCamera[MAX_CAMERAS] = {0};
  GEV_STATUS status;
  int numCamera = 0;
  MY_CONTEXT context = {0};
  //X_VIEW_HANDLE  View = NULL;
  int turboDriveAvailable = 0;

  //set default options:
  GEVLIB_CONFIG_OPTIONS options = {0};
  GevGetLibraryConfigOptions( &options);
  //options.logLevel = GEV_LOG_LEVEL_OFF;
  //options.logLevel = GEV_LOG_LEVEL_TRACE;
  options.logLevel = GEV_LOG_LEVEL_NORMAL;
  GevSetLibraryConfigOptions( &options);

  // DISCOVER Camera(s): Get all the IP addresses of attached network cards.
  status = GevGetCameraList( pCamera, MAX_CAMERAS, &numCamera);
  ROS_INFO ("%d Dalsa camera(s) found on the network", numCamera);

  //No cameras found:
  if (numCamera == 0){ 
  	ROS_ERROR("No Dalsa cameras found");
  	return -1;
  }

  // Make sure the camera Index (0 by default or passed as a ROS param) is within range
  if (camIndex >= numCamera){
  	ROS_WARN("Dalsa camera index %d out of range. Only %d camera(s) are present. Defaulting to the first camera.", camIndex, numCamera);
	camIndex = 0;
   }

	//====================================================================
	// Connect to Camera
	int i;
	int numBuffers = NUM_BUF;
	PUINT8 bufAddress[NUM_BUF];
	GEV_CAMERA_HANDLE handle = NULL;
	UINT64 size;

	int type;
	UINT32 height = 0;
	UINT32 width = 0;
	UINT32 format = 0;
	UINT32 maxHeight = 2056; //1600; DP
	UINT32 maxWidth = 2464; //2048; DP
	UINT32 maxDepth = 2;
	UINT64 payload_size;

	UINT32 pixFormat = 0;
	UINT32 pixDepth = 0;
	UINT32 convertedGevFormat = 0;		

	//====================================================================
	// Open the camera.
	if (GevOpenCamera( &pCamera[camIndex], GevExclusiveMode, &handle)){ //this function returns status=0 in success
		ROS_ERROR("Error opening the Dalsa camera.");
		return -1;
	}

	GEV_CAMERA_OPTIONS camOptions = {0};

    // Adjust the camera interface options if desired (see the manual)
	GevGetCameraInterfaceOptions( handle, &camOptions);	
	//camOptions.heartbeat_timeout_ms = 60000;	// For debugging (delay camera timeout while in debugger)
	camOptions.heartbeat_timeout_ms = 5000;		// Disconnect detection (5 seconds)

	if(tune_streaming_threads){
		// Some tuning can be done here. (see the manual)
		camOptions.streamFrame_timeout_ms = 1001;				// Internal timeout for frame reception.
		camOptions.streamNumFramesBuffered = 4;					// Buffer frames internally.
		camOptions.streamMemoryLimitMax = 64*1024*1024;			// Adjust packet memory buffering limit.	
		camOptions.streamPktSize = 9180;						// Adjust the GVSP packet size.
		camOptions.streamPktDelay = 10;							// Add usecs between packets to pace arrival at NIC.
				
		// Assign specific CPUs to threads (affinity) - if required for better performance.
		{
			int numCpus = _GetNumCpus();
			if (numCpus > 1)
			{
				camOptions.streamThreadAffinity = numCpus-1;
				camOptions.serverThreadAffinity = numCpus-2;
			}
		}
	}

	// Write the adjusted interface options back.
	GevSetCameraInterfaceOptions( handle, &camOptions);

	// Get the GenICam FeatureNodeMap object and access the camera features.
	GenApi::CNodeMapRef *Camera = static_cast<GenApi::CNodeMapRef*>(GevGetFeatureNodeMap(handle));

    if (Camera){
		// Access some features using the bare GenApi interface methods
		try {
			//Mandatory features....
			GenApi::CIntegerPtr ptrIntNode = Camera->_GetNode("Width");
			width = (UINT32) ptrIntNode->GetValue();
			ptrIntNode = Camera->_GetNode("Height");
			height = (UINT32) ptrIntNode->GetValue();
			ptrIntNode = Camera->_GetNode("PayloadSize");
			payload_size = (UINT64) ptrIntNode->GetValue();
			GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("PixelFormat") ;
			format = (UINT32)ptrEnumNode->GetIntValue();
		}
		// Catch all possible exceptions from a node access.
		CATCH_GENAPI_ERROR(status);
	}

	if(status!=0){
		ROS_ERROR("Error accessing the Dalsa camera.");
		return -1;
	}

    //ROS_INFO("Camera ROI set for \n\tHeight = %d\n\tWidth = %d\n\tPixelFormat (val) = 0x%08x\n", height,width,format);
	maxHeight = height;
	maxWidth = width;
	maxDepth = GetPixelSizeInBytes(format);

	// Allocate image buffers
	size = maxDepth * maxWidth * maxHeight;
	size = (payload_size > size) ? payload_size : size;
	//ROS_INFO("size=%ld\n", size);

	for (i = 0; i < numBuffers; i++){	//DP: numBuffers=8
		bufAddress[i] = (PUINT8)malloc(size);
		memset(bufAddress[i], 0, size);
	}


	if (!use_synchronous_buffer_cycling){
		if ( GevInitializeTransfer( handle, Asynchronous, size, numBuffers, bufAddress)){ //this function returns status=0 in success
			ROS_ERROR("Error Initializing the Asynchronous Dalsa Camera Stream Transfer.");
			return -1;
		}
	}else{
		if ( GevInitializeTransfer( handle, SynchronousNextEmpty, size, numBuffers, bufAddress)){ //this function returns status=0 in success
			ROS_ERROR("Error Initializing the Synchronous Dalsa Camera Stream Transfer.");
			return -1;
		}		
	}


	if ( GetX11DisplayablePixelFormat( ENABLE_BAYER_CONVERSION, format, &convertedGevFormat, &pixFormat) ){ //this function returns status=0 in success
		ROS_ERROR("Error Getting the X11 Displayable Pixel Format from the Dalsa Camera.");
		return -1;
	}

	//ROS_INFO("(0) pixDepth = %u\n", pixDepth);
	//ROS_INFO("(0) pixFormat = 0x%08x\n", pixFormat);

	if (format != convertedGevFormat){
		// We MAY need to convert the data on the fly to display it: 
		if (GevIsPixelTypeRGB(convertedGevFormat)){
			// DOING THIS: Conversion to RGB888 required.
			pixDepth = 32;	// Assume 4 8bit components for color display (RGBA)
			context.format = Convert_SaperaFormat_To_X11( pixFormat);
			//ROS_INFO("(IF) pixDepth = %u\n", pixDepth);
			//ROS_INFO("(IF) pixFormat = 0x%08x\n", pixFormat);
			context.depth = pixDepth;
			context.convertBuffer = malloc((maxWidth * maxHeight * ((pixDepth + 7)/8)));
			context.convertFormat = TRUE;
		}else{
			// Converted format is MONO - generally this is handled
			// internally (unpacking etc...) unless in passthru mode.			
			pixDepth = GevGetPixelDepthInBits(convertedGevFormat);	
			context.format = Convert_SaperaFormat_To_X11( pixFormat);
			context.depth = pixDepth;							
			context.convertBuffer = NULL;
			context.convertFormat = FALSE;
		}
	}else {
		pixDepth = GevGetPixelDepthInBits(convertedGevFormat);
		context.format = Convert_SaperaFormat_To_X11( pixFormat);
		context.depth = pixDepth;
		context.convertBuffer = NULL;
		context.convertFormat = FALSE;
	}

	context.camHandle = handle;
	context.exit = FALSE;

	//START STREAMING (CONTINUOUS GRAB)!
	for (i = 0; i < numBuffers; i++){
			memset(bufAddress[i], 0, size);
	}

	if ( GevStartTransfer( handle, -1)){ //this function returns status=0 in success
		ROS_ERROR("Error Streaming the Dalsa Camera's feed.");
		return -1;
	}

	MY_CONTEXT *displayContext = &context;

    //(De-)Activate Turbo Mode if available:
	if (turbo_mode && turboDriveAvailable){
		UINT32 val = 1;
		GevGetFeatureValue(handle, "transferTurboMode", &type, sizeof(UINT32), &val);

		//Toggle ON/OFF:
		val = (val == 0) ? 1 : 0;
		GevSetFeatureValue(handle, "transferTurboMode", sizeof(UINT32), &val);
		GevGetFeatureValue(handle, "transferTurboMode", &type, sizeof(UINT32), &val);

		if (val == 1){
			ROS_INFO("TurboMode Enabled"); 	
		}else{
			ROS_INFO("TurboMode Disabled"); 	
		}	

	}else{
		if (turbo_mode && turboDriveAvailable==false){
			ROS_WARN("TurboDrive is NOT Available for this Dalsa device/pixel format combination");
		}
		//ignore all this if turbo_mode=false
	}    

    ros::Rate loop_rate(100); //just an upper limit (shoul publish at around ~20Hz)

	while(ros::ok() && (!displayContext->exit)){

		GEV_BUFFER_OBJECT *img = NULL;

		// Wait for images to be received
		status = GevWaitForNextImage(displayContext->camHandle, &img, 1000);

			if ((img != NULL) && (status == GEVLIB_OK))
			{
				if (img->status == 0)
				{
					m_latestBuffer = img->address;
					// Can the acquired buffer be displayed?
					if ( IsGevPixelTypeX11Displayable(img->format) || displayContext->convertFormat )
					{
						if (displayContext->convertFormat)
						{
							int gev_depth = GevGetPixelDepthInBits(img->format);

							// [IN] FORMAT (img): "CV_8U" (BAYRG8)
							// Convert the image to a displayable format.
							// (Note : Not all formats can be displayed properly at this time (planar, YUV*, 10/12 bit packed).
							ConvertGevImageToX11Format( img->w, img->h, gev_depth, img->format, img->address, \
													displayContext->depth, displayContext->format, displayContext->convertBuffer);
							// [OUT] FORMAT (displayContext): "CV_8UC4" (CORX11_DATA_FORMAT_RGB8888)

							// Display a X11 image window in the (supported) converted format. 
							//Display_Image( displayContext->View, displayContext->depth, img->w, img->h, displayContext->convertBuffer );

							//Forget X11: use OpenCV instead (RGBA):
							cv::Mat cv_image_color(img->h, img->w, CV_8UC4, displayContext->convertBuffer);

							//In case we want to display it in OpenCV:
							//cv::imshow("opencv window", cv_image_color);
							//cv::waitKey(3);

							//Now Publish two ROS msgs: COLOR IMG and MONO IMG:
							cv_bridge::CvImage out_msg;
							out_msg.header.stamp = ros::Time::now();
							out_msg.header.frame_id = dalsa_camera_frame;
							out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
							out_msg.image    = cv_image_color;
							img_pub.publish(out_msg.toImageMsg()); //publish color

							//OpenCV (MONO):
							if (publish_mono){
								cv::Mat cv_image_mono(img->h, img->w, CV_8U, img->address);
								out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
								out_msg.image    = cv_image_mono;
								img_pub_mono.publish(out_msg.toImageMsg()); //publish color	
							}

						}else{
							// Display the image in the (supported) received format. 
							// Display_Image( displayContext->View, img->d,  img->w, img->h, img->address );
							// I have no way of testing this and open it in OpenCV and convert to ROS.
						}
					}
				}

			}else{
				// Image had an error (incomplete (timeout/overflow/lost)).
				// Do any handling of this condition necessary.
				ROS_ERROR("Error: Incomplete/Timeout/Overflow/Lost) Dalsa camera image.");
			}

	  if (use_synchronous_buffer_cycling){
		  if (img != NULL){
			// Release the buffer back to the image transfer process.
			GevReleaseImage( displayContext->camHandle, img);
		  }
	  }

	  ros::spinOnce();
	  loop_rate.sleep();
	}	

  //abort:
  GevAbortTransfer(handle);
  //quit:
  GevStopTransfer(handle);
  context.exit = TRUE;

  status = GevFreeTransfer(handle);
  //DestroyDisplayWindow(View);

  //free pointers:
  for (i = 0; i < numBuffers; i++){	
	free(bufAddress[i]);
  }

  if (context.convertBuffer != NULL){
 	free(context.convertBuffer);
	context.convertBuffer = NULL;
  }

  GevCloseCamera(&handle);

  // Close down the API.
  GevApiUninitialize();

  // Close socket API
  _CloseSocketAPI ();	// must close API even on error  

  return 0;
}
