/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Acquires stereo images from Firewire cameras using libdc1394.
 *
 * @author Florian Echtler <echtler@in.tum.de>
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#include <string>
#include <list>
#include <iostream>
#include <algorithm>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utVision/Image.h>
#include <opencv/cv.h>

#include <dc1394/control.h>
#include <dc1394/conversions.h>


#define NUM_DMA_BUFFERS 4
#define MAX_PORTS 1


// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.firewirecamera.DC1394FrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;


namespace Ubitrack { namespace Drivers {


/*-----------------------------------------------------------------------
 *  Prints the type of format to standard out
 *-----------------------------------------------------------------------*/
void print_format( uint32_t format )
{
#define print_case(A) case A: LOG4CPP_INFO( logger, #A ""); break;

    switch( format ) {
			print_case(DC1394_VIDEO_MODE_160x120_YUV444);
			print_case(DC1394_VIDEO_MODE_320x240_YUV422);
			print_case(DC1394_VIDEO_MODE_640x480_YUV411);
			print_case(DC1394_VIDEO_MODE_640x480_YUV422);
			print_case(DC1394_VIDEO_MODE_640x480_RGB8);
			print_case(DC1394_VIDEO_MODE_640x480_MONO8);
			print_case(DC1394_VIDEO_MODE_640x480_MONO16);
			print_case(DC1394_VIDEO_MODE_800x600_YUV422);
			print_case(DC1394_VIDEO_MODE_800x600_RGB8);
			print_case(DC1394_VIDEO_MODE_800x600_MONO8);
			print_case(DC1394_VIDEO_MODE_1024x768_YUV422);
			print_case(DC1394_VIDEO_MODE_1024x768_RGB8);
			print_case(DC1394_VIDEO_MODE_1024x768_MONO8);
			print_case(DC1394_VIDEO_MODE_800x600_MONO16);
			print_case(DC1394_VIDEO_MODE_1024x768_MONO16);
			print_case(DC1394_VIDEO_MODE_1280x960_YUV422);
			print_case(DC1394_VIDEO_MODE_1280x960_RGB8);
			print_case(DC1394_VIDEO_MODE_1280x960_MONO8);
			print_case(DC1394_VIDEO_MODE_1600x1200_YUV422);
			print_case(DC1394_VIDEO_MODE_1600x1200_RGB8);
			print_case(DC1394_VIDEO_MODE_1600x1200_MONO8);
			print_case(DC1394_VIDEO_MODE_1280x960_MONO16);
			print_case(DC1394_VIDEO_MODE_1600x1200_MONO16);

		default:
			dc1394_log_error("Unknown format\n");
			//exit(1);
    }

}


/**
 * @ingroup vision_components
 * Pushes images from one camera using libdc1394.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c ColorOutput push port of type Ubitrack::Vision::Measurement::ImageMeasurement.
 * \c GreyOutput  push port of type Ubitrack::Vision::Measurement::ImageMeasurement.
 */
class DC1394FrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	DC1394FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~DC1394FrameGrabber();

	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();

protected:
	// thread main loop
	void ThreadProc();

	// helper functions
	int  camera_setup();
	void camera_cleanup();
	void process_frame();
	boost::tuple< dc1394video_mode_t, dc1394framerate_t, unsigned, unsigned, float > findBestMode();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement >* m_colorPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement >* m_greyPort;

	// camera data structures
	dc1394camera_t* m_camera;
	dc1394video_frame_t* m_camera_frame;

	unsigned int m_numCameras;
	unsigned int m_width, m_height;
	unsigned int m_busSpeed;
	float m_maxFramerate;

	int m_mode;
	unsigned int m_camid;
	int m_shutter;
	int m_gain;
	unsigned int m_auto_exp;
	unsigned int m_divisor;
	unsigned int m_divisor_count;
	int m_deBayer;

};


void DC1394FrameGrabber::stop()
{
 	LOG4CPP_TRACE( logger, "Stopping thread..." );

	if ( m_running )
	{
		LOG4CPP_TRACE( logger, "Thread was running" );

		if ( m_Thread )
		{
			m_bStop = true;
			m_Thread->join();
		}
		m_running = false;
	}
}


void DC1394FrameGrabber::start()
{
 	LOG4CPP_TRACE( logger, "Starting thread..." );

	if ( !m_running )
	{
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &DC1394FrameGrabber::ThreadProc, this ) ) );
		m_running = true;
	}
}


DC1394FrameGrabber::DC1394FrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_bStop( true )
	, m_camera_frame(new dc1394video_frame_t())
	, m_numCameras( 0 )
	, m_width( 640 )
	, m_height( 480 )
	, m_busSpeed( 400 )
	, m_maxFramerate( 30.0 )
	, m_mode( -1 )
	, m_camid( 0 )
	, m_shutter( -1 )
	, m_gain( 0 )
	, m_auto_exp( 100 )
	, m_divisor( 1 )
	, m_divisor_count( 0 )
	, m_deBayer( 0 )
{
	subgraph->m_DataflowAttributes.getAttributeData( "Shutter", m_shutter );
	subgraph->m_DataflowAttributes.getAttributeData( "AutoExp", m_auto_exp );
	subgraph->m_DataflowAttributes.getAttributeData( "Gain", m_gain );
	subgraph->m_DataflowAttributes.getAttributeData( "Divisor", m_divisor );
	subgraph->m_DataflowAttributes.getAttributeData( "DeBayer", m_deBayer );
	subgraph->m_DataflowAttributes.getAttributeData( "Mode", m_mode );
	subgraph->m_DataflowAttributes.getAttributeData( "CamId", m_camid );
	subgraph->m_DataflowAttributes.getAttributeData( "BusSpeed", m_busSpeed );
	subgraph->m_DataflowAttributes.getAttributeData( "MaxFramerate", m_maxFramerate );
	
	// (daniel) added for compatibility with trackman pattern, but it's inconsistent, as resolution attribute on 'Camera' node does same thing
	subgraph->m_DataflowAttributes.getAttributeData( "SizeX", m_width ); 
	subgraph->m_DataflowAttributes.getAttributeData( "SizeY", m_height );
	
	// read parameters ### works only for a single camera, but how are the cameras named in a multi-camera setup? And who uses such a thing anyway?
	// Why not use multiple instance of a single-camera framegrabber pattern?
	Graph::UTQLSubgraph::NodePtr camNode = subgraph->getNode( "Camera" );
	if ( !camNode )
		UBITRACK_THROW( "No 'Camera' node" );
	if ( camNode->hasAttribute( "resolution" ) ) 
	{
		try 
		{
			std::string res = camNode->getAttribute( "resolution" ).getText();
			std::istringstream resStream( res );
			resStream >> m_width;
			resStream >> m_height;
		}
		catch( ... ) 
		{
			UBITRACK_THROW( "Invalid value for attribute 'resolution'" );
		}
	}

 	LOG4CPP_INFO( logger, "Set camera resolution: " << m_width << "x" << m_height );

 	m_camera_frame->color_coding = DC1394_COLOR_CODING_RGB8;
	LOG4CPP_INFO( logger, "Create pair of color/greyscale output ports..." );
	m_colorPort = new Dataflow::PushSupplier< Measurement::ImageMeasurement > ( "ColorOutput", *this );
	m_greyPort = new Dataflow::PushSupplier< Measurement::ImageMeasurement > ( "Output", *this );

	stop();
}


DC1394FrameGrabber::~DC1394FrameGrabber()
{
 	stop();
	delete m_colorPort;
	delete m_greyPort;
}


void DC1394FrameGrabber::camera_cleanup()
{
	dc1394_video_set_transmission( m_camera, DC1394_OFF );
	dc1394_capture_stop( m_camera );
	dc1394_camera_free( m_camera );
}


int DC1394FrameGrabber::camera_setup()
{
 	LOG4CPP_INFO( logger, "Setting up camera #" << m_camid << ".." );
 	
 	// determine which mode and framerate to use, based on configuration and camera support
 	dc1394video_mode_t bestVideoMode;
 	dc1394framerate_t bestFramerate;
 	unsigned bestVideoModeWidth;
 	unsigned bestVideoModeHeight;
 	float bestFramerateFloat;
 	boost::tie( bestVideoMode, bestFramerate, bestVideoModeWidth, bestVideoModeHeight, bestFramerateFloat ) =
 		DC1394FrameGrabber::findBestMode();
 	bool bIsFormat7 = bestVideoMode >= DC1394_VIDEO_MODE_FORMAT7_MIN && bestVideoMode <= DC1394_VIDEO_MODE_FORMAT7_MAX;

	//### try to reset camera
	// dc1394_camera_set_power( m_camera, DC1394_OFF );
	// Util::sleep (5);
	// dc1394_camera_set_power( m_camera, DC1394_ON );
	// dc1394_camera_reset( m_camera );
	//dc1394_cleanup_iso_channels_and_bandwidth( m_camera );


	// It should be possible to set by attribute the following stuff:
	// See also comment below
	// * video mode
	// * framerate
	// * manual/auto shutter
	// * exposure (for auto shutter only)
	// * shutter (for manual shutter only)
	
	// Use this to switch to manual settings. having this, you can use e.g. coriander to tune your
	// camera before starting or during runtime of the Ubitrack dataflow.
	
	// err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_AUTO );
	// LOG4CPP_INFO( logger, "DC1394_FEATURE_WHITE_BALANCE error code: " << err );
	// err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO );
	// LOG4CPP_INFO( logger, "DC1394_FEATURE_SHUTTER error code: " << err );
	// err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO );
	// LOG4CPP_INFO( logger, "DC1394_FEATURE_GAIN error code: " << err );

	// This is just a coding example to see how to deal with features.
	// All features should be parametrizable by component attributes, with proper error handling here!
	// In my opinion, the driver should not have much intelligence. If a feature value or capture mode or ... cannot be
	// set, it should just throw an exception. People can use other tools, e.g. 'coriander' to find out what the camera
	// supports and what not!
	dc1394error_t err;
	dc1394bool_t avail;
	dc1394_feature_is_present( m_camera, DC1394_FEATURE_BRIGHTNESS, &avail );
	if ( avail )
	{
		err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_BRIGHTNESS set to manual mode: " << err );

		uint32_t min, max;
		err = dc1394_feature_get_boundaries( m_camera, DC1394_FEATURE_BRIGHTNESS, &min, &max );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_BRIGHTNESS boundaries: " << err << ", min/max: " << min << "/" << max );
		// Unfortunately, AVT does not support this...
		err = dc1394_feature_has_absolute_control( m_camera, DC1394_FEATURE_BRIGHTNESS, &avail );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_BRIGHTNESS has absolute boundaries: " << err << ", value: " << avail );
		float fMin, fMax;
		err = dc1394_feature_get_absolute_boundaries( m_camera, DC1394_FEATURE_BRIGHTNESS, &fMin, &fMax );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_BRIGHTNESS absolute boundaries: " << err << ", min/max: " << fMin << "/" << fMax );
		err = dc1394_feature_set_value( m_camera, DC1394_FEATURE_BRIGHTNESS, 0 );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_BRIGHTNESS set feature to 0, status: " << err );
	}
	
	// This is only relevant for auto-shutter, see DC1394_FEATURE_SHUTTER
	if ( m_shutter < 0 ) 
	{
		err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_SHUTTER set to automatic mode: " << err );
		//###		err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO );
		//		LOG4CPP_INFO( logger, "DC1394_FEATURE_EXPOSURE set to automatic mode: " << err );
		err = dc1394_feature_set_value( m_camera, DC1394_FEATURE_EXPOSURE, m_auto_exp );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_EXPOSURE set feature to " << m_auto_exp << ", status: " << err );
	}
	else
	{
		err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_SHUTTER set to manual mode: " << err );
		err = dc1394_feature_set_value( m_camera, DC1394_FEATURE_SHUTTER, (unsigned int) m_shutter );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_SHUTTER set feature to " << m_shutter << ", status: " << err );
	}
	
	err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL );
	LOG4CPP_INFO( logger, "DC1394_FEATURE_GAMMA set to manual mode: " << err );
	err = dc1394_feature_set_value( m_camera, DC1394_FEATURE_GAMMA, 0 );
	LOG4CPP_INFO( logger, "DC1394_FEATURE_GAMMA set feature to 0, status: " << err );

	if ( m_gain < 0 ) 
	{
		err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_GAIN set to automatic mode: " << err );
	}
	else 
	{
		err = dc1394_feature_set_mode( m_camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_GAIN set to manual mode: " << err );
		err = dc1394_feature_set_value( m_camera, DC1394_FEATURE_GAIN, (unsigned int) m_gain );
		LOG4CPP_INFO( logger, "DC1394_FEATURE_GAIN set feature to " << m_gain << ", status: " << err );
	}

	dc1394_video_set_transmission( m_camera, DC1394_OFF );

	if (m_busSpeed <= 100) {
		dc1394_video_set_iso_speed( m_camera, DC1394_ISO_SPEED_100 );
	} else if (m_busSpeed <= 200) {
		dc1394_video_set_iso_speed( m_camera, DC1394_ISO_SPEED_200 );
	} else {
		dc1394_video_set_iso_speed( m_camera, DC1394_ISO_SPEED_400 );
	}

	LOG4CPP_INFO( logger, "Setting video mode " << bestVideoMode << 
		" (" << bestVideoModeWidth << "x" << bestVideoModeHeight << ")" )
		
	if ( dc1394_video_set_mode( m_camera, bestVideoMode ) != DC1394_SUCCESS )
	{
		camera_cleanup( );
		LOG4CPP_ERROR( logger, "Error during camera initialization. Check that the selected mode is supported." );
		return 0;
	}
	
	// for format7, get real image size instead of relying on configuration parameter
	if ( bIsFormat7 )
	{
		dc1394_format7_get_image_size( m_camera, bestVideoMode, &m_width, &m_height );
		LOG4CPP_INFO( logger, "Using format7. Image size: " << m_width << "x" << m_height );
	}
	else
	{
		m_width = bestVideoModeWidth;
		m_height = bestVideoModeHeight;
	}
	
	// set video framerate only if not in format7 (for format7, this works differently)
	if (!bIsFormat7 && dc1394_video_set_framerate( m_camera, bestFramerate ) != DC1394_SUCCESS)
	{
		camera_cleanup( );
		LOG4CPP_ERROR( logger, "Error during camera initialization. Check that the framerate is supported." );
		return 0;
	}

	if (dc1394_capture_setup( m_camera, NUM_DMA_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT ) != DC1394_SUCCESS)
	{
		camera_cleanup( );
		LOG4CPP_ERROR( logger, "Error during camera initialization. Check that 640x480_YUV411@30FPS is supported." );
		return 0;
	}
 
	if (dc1394_video_set_transmission( m_camera, DC1394_ON ) != DC1394_SUCCESS)
	{
		camera_cleanup( );
		LOG4CPP_ERROR( logger, "Unable to start ISO transmission." );
		return 0;
	}
	
	LOG4CPP_INFO( logger, "Camera #" << m_camid << " setup successful." );
	return 1;
}


/** convert YUV411 image data to greyscale - this function is missing in libdc1394 */
void dc1394_YUV411_to_MONO8( uint8_t* src, uint8_t* dest, uint32_t width, uint32_t height )
{
	register int i = (width*height) + ( (width*height) >> 1 ) -1;
	register int j = (width*height)                           -1;
	register int y0, y1, y2, y3;
  
	while (i >= 0) {
		y3 = (uint8_t) src[i--];
		y2 = (uint8_t) src[i--]; i--;
		y1 = (uint8_t) src[i--];
		y0 = (uint8_t) src[i--]; i--;
		dest[j--] = y3;
		dest[j--] = y2;
		dest[j--] = y1;
		dest[j--] = y0;
	}
}


/** retrieve frame from camera, convert to RGB or greyscale, send through port */
void DC1394FrameGrabber::process_frame( )
{
	dc1394error_t err;
	dc1394video_frame_t* frame;

	/* retrieve raw YUV411 data */
	dc1394_capture_dequeue( m_camera, DC1394_CAPTURE_POLICY_WAIT, &frame );
	
	/* if we are lagging behind, dump a frame to catch up */
	if (frame->frames_behind > 0) {
		LOG4CPP_INFO( logger, "dumping frame (lag: " << frame->frames_behind << " frame(s))" );
		dc1394_capture_enqueue( m_camera, frame );
		dc1394_capture_dequeue( m_camera, DC1394_CAPTURE_POLICY_WAIT, &frame );
	}

	/* store the last frame's timestamp */
	Measurement::Timestamp time( frame->timestamp * 1000 );

	boost::shared_ptr< Image > pColorImage;
	boost::shared_ptr< Image > pGreyImage;
	bool has_greyimage = false;
	bool has_colorimage = false;

    switch( frame->color_coding ) {
		case DC1394_COLOR_CODING_MONO8:
			pGreyImage.reset( new Image( m_width, m_height, 1 ) );
			pGreyImage->origin = 0;
			has_greyimage = true;
			break;
		case DC1394_COLOR_CODING_YUV411:
		case DC1394_COLOR_CODING_YUV422:
		case DC1394_COLOR_CODING_YUV444:
			pColorImage.reset( new Image( m_width, m_height, 3 ) );
			pColorImage->origin = 0;
			pColorImage->channelSeq[0]='B';
			pColorImage->channelSeq[1]='G';
			pColorImage->channelSeq[2]='R';
			err=dc1394_convert_frames(frame, m_camera_frame);
			if (err != DC1394_SUCCESS) {
				LOG4CPP_INFO( logger, "Failed to convert frame");
			}
			memcpy( pColorImage->imageData, m_camera_frame->image, m_width * m_height * 3 );
			//pColorImage->imageData = (char *)(m_camera_frame->image);
			has_colorimage = true;
			break;
		case DC1394_COLOR_CODING_RGB8:
			pColorImage.reset( new Image( m_width, m_height, 3 ) );
			pColorImage->origin = 0;
			pColorImage->channelSeq[0]='R';
			pColorImage->channelSeq[1]='G';
			pColorImage->channelSeq[2]='B';
			memcpy( pColorImage->imageData, frame->image, m_width * m_height * 3 );
			has_colorimage = true;
			break;
		case DC1394_COLOR_CODING_MONO16:
		case DC1394_COLOR_CODING_RGB16:
		case DC1394_COLOR_CODING_MONO16S:
		case DC1394_COLOR_CODING_RGB16S:
		case DC1394_COLOR_CODING_RAW8:
		case DC1394_COLOR_CODING_RAW16:
		default:
			LOG4CPP_INFO(logger, "Unhandled ColorCoding.");
    }

    /*
	if ( m_deBayer )
	{
		dc1394_bayer_decoding_8bit( frame->image, reinterpret_cast< unsigned char* >( pColorImage->imageData ), m_width, m_height,
			DC1394_COLOR_FILTER_RGGB, DC1394_BAYER_METHOD_HQLINEAR);
	}
	*/

	if (m_colorPort->isConnected()) {
		if (has_colorimage) {
			boost::shared_ptr< Image > bgr_img = pColorImage->CvtColor( CV_RGB2BGR, 3 );
			m_colorPort->send( Measurement::ImageMeasurement( time, bgr_img ) );
		} else if(has_greyimage) {
			pColorImage = pGreyImage->CvtColor( CV_GRAY2BGR, 0 );
			m_colorPort->send( Measurement::ImageMeasurement( time, pColorImage ) );
		}
	}

	if (m_greyPort->isConnected()) {
		if (has_greyimage) {
			memcpy( pGreyImage->imageData, frame->image, m_width * m_height );
			m_greyPort->send( Measurement::ImageMeasurement( time, pGreyImage ) );
		} else if(has_colorimage) {
			pGreyImage = pColorImage->CvtColor( CV_RGB2GRAY, 1 );
			m_greyPort->send( Measurement::ImageMeasurement( time, pGreyImage ) );
		}
	}

	dc1394_capture_enqueue( m_camera, frame );
}


/** helper function to sort the camera array */
bool cmp( dc1394camera_t* a, dc1394camera_t* b )
{
	return a->guid > b->guid;
}


void DC1394FrameGrabber::ThreadProc()
{
	std::cout << "framegrabber thread starting..." << std::endl;
	
	dc1394_t * d;
    dc1394camera_list_t * list;
	dc1394error_t err;

    d = dc1394_new ();
	LOG4CPP_TRACE( logger, "dc1394 interface initialized" );
	err = dc1394_camera_enumerate (d, &list);
	LOG4CPP_TRACE( logger, "cameras enumerated, status is " << err );

    if (list->num == 0) {
        LOG4CPP_ERROR( logger, "No cameras found" );
		return;
    }
	
	m_numCameras = list->num;
	LOG4CPP_INFO( logger, m_numCameras << " cameras found" );

    if (list->num <= m_camid) {
        LOG4CPP_ERROR( logger, "Invalid CamId specified " << m_camid );
		return;
    }


    m_camera = dc1394_camera_new (d, list->ids[m_camid].guid);
    if (!m_camera) {
        dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[m_camid].guid);
		return;
    }
    dc1394_camera_free_list (list);

	LOG4CPP_INFO( logger, "Using camera with GUID " << m_camera->guid );

	
	//	setup camera
	if (!camera_setup()) return;

	while ( !m_bStop )
	{
		/* acquire frames from all cameras and wait for completion */
		process_frame();
	}

	camera_cleanup();
}


boost::tuple< dc1394video_mode_t, dc1394framerate_t, unsigned, unsigned, float > DC1394FrameGrabber::findBestMode( )
{
	dc1394video_mode_t bestVideoMode = DC1394_VIDEO_MODE_FORMAT7_0;
	dc1394framerate_t  bestFramerate = DC1394_FRAMERATE_1_875;
	float bestFramerateFloat = 0.0f;
	unsigned bestVideoModeWidth = 0;
	unsigned bestVideoModeHeight = 0;
	
	
	// find suitable video mode
	dc1394video_modes_t video_modes;
	dc1394_video_get_supported_modes( m_camera, &video_modes );
	LOG4CPP_INFO( logger, "Available modes for this camera: " << video_modes.num );
	for ( unsigned int i = 0; i < video_modes.num; i++ ) 
	{
		unsigned width = 0;
		unsigned height = 0;
		dc1394color_coding_t colorCoding;
		print_format( video_modes.modes[ i ] );

		if ( dc1394_get_image_size_from_video_mode( m_camera, video_modes.modes[ i ], &width, &height ) == DC1394_SUCCESS )
		{
			LOG4CPP_INFO( logger, "Mode: " << video_modes.modes[ i ] << " (" << width << "x" << height << ")" );
			
			if ((m_mode >= 0) && (i == (unsigned int)m_mode)) {
				bestVideoMode = video_modes.modes[ i ];
				bestFramerate = DC1394_FRAMERATE_1_875;
				bestVideoModeWidth = width;
				bestVideoModeHeight = height;
			} else {
				dc1394_get_color_coding_from_video_mode( m_camera, video_modes.modes[ i ], &colorCoding );

				if ( width == m_width && height == m_height && colorCoding == DC1394_COLOR_CODING_MONO8 &&
					bestVideoMode == DC1394_VIDEO_MODE_FORMAT7_0 )
				{
					bestVideoMode = video_modes.modes[ i ];
					bestFramerate = DC1394_FRAMERATE_1_875;
					bestVideoModeWidth = width;
					bestVideoModeHeight = height;
				}
			}
		}
		else
			LOG4CPP_INFO( logger, "Mode: " << video_modes.modes[ i ] << " (unknown size)" );
			
		dc1394framerates_t framerates;
		dc1394_video_get_supported_framerates( m_camera, video_modes.modes[i], &framerates );
		LOG4CPP_INFO( logger, "Available framerates for this camera: " << framerates.num );
		for ( unsigned int j = 0; j < framerates.num; j++ ) 
		{
			float framerateFloat;
			if ( dc1394_framerate_as_float( framerates.framerates[j], &framerateFloat ) == DC1394_SUCCESS )
			{
				LOG4CPP_INFO( logger, "- Framerate: " << framerates.framerates[j] << " (" << framerateFloat << ")" );
			

				if ( framerateFloat <= m_maxFramerate && bestVideoMode == video_modes.modes[ i ] && framerateFloat > bestFramerateFloat )
				{
					bestFramerate = framerates.framerates[ j ]; 
					bestFramerateFloat = framerateFloat;
				}
			}
			else
				LOG4CPP_INFO( logger, "- Framerate: " << framerates.framerates[j] );
		}

		// manually selected
		if ((m_mode >= 0) && (i == (unsigned int)m_mode)) {
			LOG4CPP_INFO( logger, "Manually Selected Mode " << m_mode << " Framerate: " << bestFramerateFloat);
			break;
		}


	}

	// Just for information
	// Just for information...
	/*
	dc1394featureset_t features;
	err = dc1394_feature_get_all( m_camera, &features);
	for ( int i = 0; i < DC1394_FEATURE_NUM; i++ ) 
	{
		dc1394feature_info_t feature = features.feature[i];
		LOG4CPP_INFO( logger, "Feature: ID: " << feature.id << ", available: " << feature.available << ", abolute_capable: " << feature.absolute_capable << ", on_off_capable: " << feature.on_off_capable << ", is_on: " << feature.is_on );

		// Get modes per feature
		dc1394feature_modes_t modes;
		err = dc1394_feature_get_modes( m_camera, feature.id, &modes );
		LOG4CPP_INFO( logger, "- Num: " << modes.num );
		for ( int j = 0; j < DC1394_FEATURE_MODE_NUM; j++ ) 
		{
			LOG4CPP_INFO( logger, "- Mode: " << modes.modes[j] );
		}
	}
	*/
	LOG4CPP_INFO( logger, "Selected Mode:");
	print_format( bestVideoMode );
	return boost::make_tuple( bestVideoMode, bestFramerate, bestVideoModeWidth, bestVideoModeHeight, bestFramerateFloat );
}





} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::DC1394FrameGrabber > ( "DC1394FrameGrabber" );
}


