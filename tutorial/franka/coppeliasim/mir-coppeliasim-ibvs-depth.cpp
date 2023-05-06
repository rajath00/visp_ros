
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{

  cv::Mat depth;
  cv::Mat desired_depth;
  cv::Mat image;
  cv::Mat desired_image;
  
public:
  ImageConverter()
    // : it_(nh_)
  {
    // // Subscrive to input video feed and publish output video feed
    // depth_sub_ = it_.subscribe("/camera/depth", 1,
    //   &ImageConverter::imageCb, this);
    // desired_depth_sub_ = it_.subscribe("/desired_camera/depth", 1,
    //   &ImageConverter::desired_imageCb, this);
    
    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void desired_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr1;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr1 = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    desired_depth = cv_ptr->image;

    cv::Mat img3;
    img3 = cv_ptr1->image;
    double max_val;

    cv::minMaxIdx(img3,NULL,&max_val);
    std::cout<<max_val<<std::endl;
    img3.convertTo(desired_image,-1,255.0/max_val,0);

  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr1;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr1 = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // std::cout<<cv_ptr->encoding<<std::endl;

    cv::Mat img3;
    depth = cv_ptr->image;
    img3 = cv_ptr1->image;
    double max_val;

    cv::minMaxIdx(img3,NULL,&max_val);
    std::cout<<max_val<<std::endl;
    img3.convertTo(image,-1,255.0/max_val,0);

    // std::cout << "I = " << std::endl << " " << image << std::endl << std::endl;
    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, img3);
    // cv::waitKey(3);

  }
    cv::Mat get_current_depth()
  {
    return depth;
  }
    cv::Mat get_current_image()
  {
    return image;
  }

  cv::Mat get_desired_depth()
  {
    return desired_depth;
  }
  cv::Mat get_desired_image()
  {
    return desired_image;
  }
};

int
main( int argc, char **argv )
{
  int opt_quad_decimate          = 2;
  bool opt_verbose               = true;
  bool opt_plot                  = true;
  bool opt_adaptive_gain         = true;
  bool opt_task_sequencing       = false;
  double convergence_threshold   = 0.0005;

try
{
	// define the ros node
	ros::init(argc, argv , "vsip_ros");  
  std::cout<<"created node"<<std::endl;
	ros::NodeHandlePtr n =boost::make_shared<ros::NodeHandle>();
	ros::Rate loop_rate(1000);
	ros::spinOnce();
	
    // define publishers
    std::string m_topic_end_effector_vel = "/fakeFCI/end_effector_vel";
    std::cout<<"created vel published"<<std::endl;
    ros::NodeHandlePtr m = boost::make_shared< ros::NodeHandle >();
    ros::Publisher m_pub_end_effector_vel;
    m_pub_end_effector_vel = m->advertise<geometry_msgs::Twist >(m_topic_end_effector_vel, 1);
    
	// define publisher
    std::string m_topic_feature_error;
    m_topic_feature_error = "/feature_error";
    std::cout<<"created error published"<<std::endl;
    ros::NodeHandlePtr l = boost::make_shared< ros::NodeHandle >();
    ros::Publisher m_pub_feature_error;
    m_pub_feature_error = l->advertise<std_msgs::Float64 >(m_topic_feature_error, 1);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber depth_sub_;
    image_transport::Subscriber desired_depth_sub_;
    ImageConverter ic;
    depth_sub_ = it.subscribe("/camera/depth", 1,
    &ImageConverter::imageCb,&ic);
    desired_depth_sub_ = it.subscribe("/desired_camera/depth", 1,
    &ImageConverter::desired_imageCb,&ic);

    std::cout<<"created image subscriber"<<std::endl;
//-------------------------------------------------------------------------------------------------------------------------------

  vpDisplay::vpScaleType scale = vpDisplay::SCALE_DEFAULT;
	// define the Image grabber
	vpImage< unsigned char> I;
  vpImage< unsigned char> J;
	vpROSGrabber g;
  cv::Mat im = ic.get_current_image();
  cv::Mat depth = ic.get_current_depth();
  vpCameraParameters cam;
  try{
	g.setImageTopic("/camera/image/");
	g.setCameraInfoTopic("/camera/depth/camera_info");
	g.open(argc,argv);
	g.acquire(J);
  std::cout<<"g.open()"<<std::endl;
  vpImageConvert::convert(im,I);
  std::cout<<"vpConvert"<<std::endl;
  g.getCameraInfo( cam );
  std::cout<<"vpConvert"<<std::endl;
  std::cout << cam << std::endl;
  vpDisplayOpenCV dc( J, I.getWidth(), I.getHeight(), "Current Depth image");
}
catch( const vpException &e )
  {
    std::cout << "Error i ros grabber " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  cv::Mat desired_depth = ic.get_desired_depth();
  cv::Mat desired_im = ic.get_desired_image();
	// image grabber of the reference image
	vpImage< unsigned char> Desired_I;
  vpImage< unsigned char> Desired_J;
	vpROSGrabber h;
  h.setImageTopic("/desired_camera/image/");
	h.setCameraInfoTopic("/desired_camera/depth/camera_info");
	h.open(argc,argv);
  h.acquire(Desired_J);
  vpImageConvert::convert(desired_im,Desired_I);
  vpCameraParameters desired_cam;
  h.getCameraInfo( desired_cam );

    
  vpDisplayOpenCV dd( Desired_J, Desired_I.getWidth(), Desired_I.getHeight(), "Desired Depth image",scale);

	// creating visual features
	int feature_total = depth.total();
	std::vector <vpFeaturePoint> p(feature_total), pd(feature_total);
	
	// setting up the control law properties
	vpServo task;
	
	for (unsigned int i=0;i<p.size();i++)
	{
		task.addFeature(p[i], pd[i]);
	}
	
	task.setServo( vpServo::EYEINHAND_CAMERA);
	task.setInteractionMatrixType(vpServo::CURRENT);

    if ( opt_adaptive_gain )
    {
      std::cout << "Enable adaptive gain" << std::endl;
      vpAdaptiveGain lambda( 4, 1.2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
      task.setLambda( lambda );
    }
    else
    {
      task.setLambda( 1.2 );
    }

    // vpPlot *plotter = nullptr;

    // if ( opt_plot )
    // {
    //   plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
    //                         "Real time curves plotter" );
    //   plotter->setTitle( 0, "Visual features error" );
    //   plotter->setTitle( 1, "Camera velocities" );
    //   plotter->initGraph( 0, 8 );
    //   plotter->initGraph( 1, 6 );
    //   }

	bool final_quit = false;
	bool has_converged = false;
	bool servo_started = false;
	double sim_time            = 0;
    double sim_time_prev       = sim_time;
    double sim_time_init_servo = sim_time;
    double sim_time_img        = sim_time;
    double wait_time = 0.02;

    int check = 0;
    std::cout<< " initializing the desired depth map"<<std::endl;

    cv::Size size = depth.size();
    int height = size.height;
    int width = size.width;

    try
    {
    for (unsigned int i=0;i<height;i++)
	{
        for(unsigned int j=0;j<width;j++)
        {
            pd[check].set_x(i);
            pd[check].set_y(j);
            pd[check].set_Z(desired_depth.at<float>(i,j));
            // float depth_pixel = M.at<float>(i,j);
            // std::cout<<depth_pixel<<std::endl;
            // std::vector<float> *vf2 = new std::vector<float>;
            // vf2->assign()
            // uint8_t *depth_pixel_array = reinterpret_cast<uint8_t *>(&depth_pixel);
            // std::vector<uint8_t> depth_pixel_vector (depth_pixel_array,depth_pixel_array+4);
            // std::cout<<depth_pixel_vector<<" ";
            check++;
            
        }
		
	}
    }
    catch (const vpException &e)
    {
        std::cout<< " pd initialize error";
        return EXIT_FAILURE;
    }
    std::cout<< " pd initialize over"<<std::endl;
    

	while( !final_quit)
	{
		sim_time = sim_time + wait_time;
    im = ic.get_current_image();
    depth = ic.get_current_depth();
		vpImageConvert::convert(im,I);
		vpDisplay::display(I);



        // h.acquire(Desired,sim_time_img);
        // vpDisplay::display(Desired);

        int check = 0;
        for (unsigned int i=0;i<height;i++)
	    {
            for(unsigned int j=0;j<width;j++)
            {
            p[check].set_x(i);
            p[check].set_y(j);
            p[check].set_Z(depth.at<float>(i,j));
            check++;
            }
		
	    }
        vpColVector v_c( 6 );
        v_c = task.computeControlLaw();
        std::cout<< "task dimension : "<<task.getDimension()<<std::endl;
        vpServo::vpServoPrintType disp = vpServo::INTERACTION_MATRIX;
        // std::cout<< "interaction matrix : "<<interac.getRow(0)<<std::endl;
        vpServoDisplay::display( task, cam, I );

        // vpServoDisplay::display( task, desired_cam, Desired );

        // if ( opt_plot )
        // {
        //   plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
        //   plotter->plot( 1, static_cast< double >( sim_time ), v_c );
        // }

        if ( opt_verbose )
        {
          std::cout << "v_c: " << v_c.t() << std::endl;
        }

        double error = task.getError().sumSquare();
        std::stringstream ss;
        ss << "||error||: " << error;
        vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );



        std_msgs::Float64 error_msg;
        error_msg.data = error;
        m_pub_feature_error.publish(error_msg);


        std::cout<<" error_published"<<std::endl;
        if ( opt_verbose )
          std::cout << ss.str() << std::endl;

        if ( !has_converged && error < convergence_threshold )
        {
          has_converged = true;
          std::cout << "Servo task has converged"
                    << "\n";
          vpDisplay::displayText( I, 100, 20, "Servo task has converged", vpColor::red );
        //   final_quit = true;
        }

        {
          
          geometry_msgs::Twist vel_msg;
          vel_msg.linear.x  = v_c[0];
          vel_msg.linear.y  = v_c[1];
          vel_msg.linear.z  = v_c[2];
          vel_msg.angular.x = v_c[3];
          vel_msg.angular.y = v_c[4];
          vel_msg.angular.z = v_c[5];
          m_pub_end_effector_vel.publish( vel_msg );

        }
        vpMouseButton::vpMouseButtonType button;
        if ( vpDisplay::getClick( I, button, false ) )
        {
            switch ( button )
            {
            case vpMouseButton::button1:
            // send_velocities = !send_velocities;
            break;

            case vpMouseButton::button3:
            final_quit = true;
            v_c        = 0;
            break;

            default:
            break;
            }
        }

        vpDisplay::flush( I );
        // vpDisplay::flush(Desired);
        // Slow down the loop to simulate a camera at 50 Hz
    } // end while                        
        if ( !final_quit )
        {
        while ( !final_quit )
        {
            im = ic.get_current_image();
		        vpImageConvert::convert(im,I);
            vpDisplay::display( I );

            vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
            vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

            if ( vpDisplay::getClick( I, false ) )
            {
            final_quit = true;
            }

            vpDisplay::flush( I );
            // vpDisplay::flush( Desired );
        }
        }

    }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}