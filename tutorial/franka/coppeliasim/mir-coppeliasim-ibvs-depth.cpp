
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

// Declared image variables that have to be subscribed from the depth camera
  int height;
  int width;
  cv::Mat depth;
  cv::Mat desired_depth;
  cv::Mat image;
  cv::Mat desired_image;
  cv::Mat A,B;
  vpMatrix Lz; // Computing the Interaction matrix
  std::vector<int> size;
  
public:
  ImageConverter(const int h, const int w)
    // : it_(nh_)
  {
    
    height = h;
    width = w;
    Lz.resize(h*w,6);
    size.push_back(h);
    size.push_back(w);
    A.create(size,CV_16S);
    B.create(size,CV_16S);
    std::cout<<"Image Transport Initialized"<<std::endl;
    // // Subscrive to input video feed and publish output video feed
    // depth_sub_ = it_.subscribe("/camera/depth", 1,
    //   &ImageConverter::imageCb, this);
    // desired_depth_sub_ = it_.subscribe("/desired_camera/depth", 1,
    //   &ImageConverter::desired_imageCb, this);
    
    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

  void desired_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    // std::cout<<"Desired imageCb function called"<<std::endl;
    cv_bridge::CvImagePtr depth_ptr;  // reading depth values
    cv_bridge::CvImagePtr image_ptr; // reading colour values

    try
    {
      depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);  // converting depth values
      image_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC1);   // converting image pixel values
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img3;

  //Storing the depth values in the cv::Mat format and image in cv::Mat format as well.
  try
  {
    desired_depth = depth_ptr->image;
    img3 = image_ptr->image;
  }
  catch(const std::exception& e)
  {
    // std::cout<<"error in Desired_imageCb"<<std::endl;
    std::cerr << e.what() << '\n';
  }

  // convert the image into greyscale.
  double max_val;
  cv::minMaxIdx(img3,NULL,&max_val);
  img3.convertTo(desired_image,-1,255.0/max_val,0);

  // std::cout<<desired_depth<<std::endl;
  //std::cout<<"inside the function "<<desired_depth.size()<<std::endl;
    
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // std::cout<<"Current imageCb function called"<<std::endl;
    cv_bridge::CvImagePtr depth_ptr; // reading depth values
    cv_bridge::CvImagePtr image_ptr; // reading colour values
    
    try
    {
     // std::cout<<"Exception in ImageCB"<<std::endl;
      depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); // converting depth values
      image_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC1); // converting image pixel values
     // std::cout<<"Exception in ImageCB2"<<std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
      //std::cout<<"Exception in ImageCB"<<std::endl;
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //std::cout<<"No Exception in ImageCB"<<std::endl;
    //Storing the depth values in the cv::Mat format and image in cv::Mat format as well.
    cv::Mat img3;
    try
    {
      depth = depth_ptr->image;
      img3 = image_ptr->image;
     // std::cout<<"No xception in storing the values"<<std::endl;
    }
    catch(const std::exception& e)
    {
      //std::cout<<"error in imageCb"<<std::endl;
      std::cerr << e.what() << '\n';
    }

    // convert the image into greyscale.
    double max_val;
    cv::minMaxIdx(img3,NULL,&max_val);
    img3.convertTo(image,-1,255.0/max_val,0);

    // std::cout<<depth<<std::endl;
    //std::cout<<"No Exception in acquiring images"<<std::endl;
    // Calculating the Interaction Matrix 
    int ksize = 1;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    cv::Sobel(depth,A,ddepth,1,0,ksize,scale,delta,cv::BORDER_DEFAULT); // A = dZ/dx
    cv::Sobel(depth,B,ddepth,0,1,ksize,scale,delta,cv::BORDER_DEFAULT); // B = dZ/dy
    // std::cout<<"No Problem in Sobel"<<std::endl;
    // cv::Size size = depth.size();
    // int height = size.height;
    // int width = size.width;
    int total = height*width;
    int count = 0;

    const std::vector<int> &_sz{height*width, 6};
    cv::Mat L(_sz, CV_32FC1, cv::Scalar::all(0));  // PRedefine the Interaction Matrix of sixe nx6
    
    // cv::Mat M();
    // std::cout<<"Problem in assigning L?"<<std::endl;
    // std::cout<<L<<std::endl;
    // std::cout<<"depth"<<std::endl;
    // std::cout<<depth<<std::endl;
    // std::cout<<"A"<<std::endl;
    // std::cout<<A<<std::endl;
    // std::cout<<"B"<<std::endl;
    // std::cout<<B<<std::endl;

    // std::cout<<"is A continuous"<< A.isContinuous();
    //  for(int i=0;i<height;i++)
    // {
    //  for(int j=0;j<width;j++)
    //  {
    //     std::cout<<"A("<<i<<","<<j<<")"<<A.at<short>(i,j)<<std::endl;
    //  }
    // }

    // for(std::size_t i = 0;i!=A.size[0]*A.size[1];i++)
    // {
    //   float *p = A.ptr<float>(i);
    //   std::cout<<*p<<" ";
    // }

    // std::cout<<"A(0,0)"<<A.at<_Float32>(0,0)<<std::endl;
    // std::cout<<"A type "<<A.type()<<std::endl; 
    // std::cout<<"L type"<<L.type()<<std::endl; 
    // cv::MatIterator_<_Float32> it, end;

    // for(it = A.begin<_Float32>(), end=A.end<_Float32>();it!=end;++it)
    // {
    //   std::cout<<*it<<" ";
    // }

    for(int i=0;i<height;i++)
    {
      float Z,a,b;
      float ab;
      float zx,zy,zz;
      for(int j=0;j<width;j++)
      {
        
        Z = depth.at<float>(i,j);
        // std::cout<<"depth"<<std::endl;
        // std::cout<<depth<<std::endl;
        // std::cout<<"Z"<<std::endl;
        // std::cout<<Z<<std::endl;

        a = A.at<short>(i,j);
        b = B.at<short>(i,j);
        ab = -(Z+(i*a)+(j*b)); // \frac{-Z+xA+yB}{Z}

        zx = (-j*Z) - (i*j*a)- ((1+(j*j))*b); //$Z_{w_{x}} = -yZ - xyA-(1+y^{2})B$
        zy = (i*Z) + (i*j*b)+ ((1+(i*i))*a);  // $Z_{w_{y}} = xZ+(1+x^{2})A+xyB$
        zz = (i*b) - (j*a);                   //$Z_{w_{y}} = xZ+(1+x^{2})A+xyB$

        // std::cout<<"(i,j) = ("<<i<<","<<j<<")"<<std::endl;
      /*$$
        \mathbf{L_{Z}} = \begin{bmatrix}
        \frac{A}{Z} & \frac{B}{Z} & \frac{-Z+xA+yB}{Z} & Z_{w_{x}} & Z_{w_{y}} & Z_{w_{z}}
        \end{bmatrix}, \tag{9}
        $$ */
        // std::cout<<"Z :"<<Z<<std::endl;
        // std::cout<<"a :"<<a<<" b: "<<b<<" ab: "<<ab<<std::endl;
        // std::cout<<"zx :"<<zx<<" zy: "<<zy<<" zz: "<<zz<<std::endl;

        cv::Mat temp = (cv::Mat_<float>(1,6)<<a/Z,b/Z,ab/Z,zx , zy, zz);
        // std::cout<<"before"<<std::endl;
        // std::cout<<temp1<<std::endl;  // create a row of vectors
        // std::cout<<"Problem here?"<<std::endl;
        // std::cout<<"after"<<std::endl;
        temp.copyTo(L.row(count)); // copy row to interaction matrix
        // std::cout<<L<<std::endl;
        // std::cout<<"-------------------------------------------"<<std::endl;
        
        // std::cout<<"L.row()"<<std::endl;
        // std::cout<<L.row(count)<<std::endl;
        // std::cout<<"L"<<std::endl;
        // std::cout<<L<<std::endl;
        // std::cout<<"Problem in copy?"<<std::endl;
        count++;
      }
    }
    
    // convert all the values to float
    for(int i = 0;i<total;i++)
    {
      for(int j=0;j<6;j++)
      {
        Lz[i][j] = L.at<float>(i,j);
      }
    }
  }

  // return current depth
    cv::Mat get_current_depth()
  {
    // std::cout<<depth<<std::endl;
    return depth;
  }

  // return current image
    cv::Mat get_current_image()
  {
    return image;
  }

  // return desired depth
  cv::Mat get_desired_depth()
  {
    return desired_depth;
  }

  // return desired image
  cv::Mat get_desired_image()
  {
    std::cout<<desired_image.size()<<std::endl;
    return desired_image;
  }
  // return the interactpm matrix
  vpMatrix get_Interaction_Matrix()
  {

    return Lz;

  }
};

//----------------------------------------------------------------

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

  /*
   1. Create ROS node
   2. Create Velocity Publisher
   3. Create Error Publisher
  */

	ros::init(argc, argv , "vsip_ros");  
  std::cout<<"created node"<<std::endl;
	ros::NodeHandlePtr n =boost::make_shared<ros::NodeHandle>();
	ros::Rate loop_rate(1000);
	// ros::spinOnce();

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	
  std::string m_topic_end_effector_vel = "/fakeFCI/end_effector_vel";
  std::cout<<"created vel publisher"<<std::endl;
  ros::NodeHandlePtr m = boost::make_shared< ros::NodeHandle >();
  ros::Publisher m_pub_end_effector_vel;
  m_pub_end_effector_vel = m->advertise<geometry_msgs::Twist >(m_topic_end_effector_vel, 1);
    
	
  std::string m_topic_feature_error;
  m_topic_feature_error = "/feature_error";
  std::cout<<"created error publisher"<<std::endl;
  ros::NodeHandlePtr l = boost::make_shared< ros::NodeHandle >();
  ros::Publisher m_pub_feature_error;
  m_pub_feature_error = l->advertise<std_msgs::Float64 >(m_topic_feature_error, 1);

  std::cout<<"---------------------------------------------"<<std::endl;


  /*
  
  1. Create ROS Grabber and Camera parameter object
  2. Read the current and desired rgb images from their ROS topics
  3. Read the camera parameters of both the current and desired cameras.
  */

  vpDisplay::vpScaleType scale = vpDisplay::SCALE_DEFAULT;
  vpImage< unsigned char> J, Desired_J;
	vpROSGrabber g, h;
  vpCameraParameters cam, desired_cam;

  try
  {
    g.setImageTopic("/camera/image/");
    g.setCameraInfoTopic("/camera/depth/camera_info");
    g.open(argc,argv);
    g.acquire(J);
    g.getCameraInfo( cam );


    h.setImageTopic("/desired_camera/image/");
    h.setCameraInfoTopic("/desired_camera/depth/camera_info");
    h.open(argc,argv);
    h.acquire(Desired_J);
    h.getCameraInfo( desired_cam );
  }
  catch( const vpException &e )
  {
    std::cout << "Error i ros grabber " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << cam << std::endl;
  const int width = J.getWidth();
  const int height = J.getHeight();
  const int total_features = width*height;

  std::cout<< "Camera resolution :"<<width<<"x"<<height<<std::endl;
  /*

  Display the rgb image
  */



  vpDisplayOpenCV dc; 
  try
  {
    dc.init(J);
    vpDisplay::setWindowPosition(J,400,100);
    vpDisplay::setTitle(J,"Current Depth image");
    // vpDisplay::display(J);
    // vpDisplayOpenCV dd( Desired_J, Desired_I.getWidth(), Desired_I.getHeight(), "Desired Depth image");

  }
  catch( const vpException &e )
  {
    std::cout << "Error i opencv display " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }


  /*
    1. Create subscriber to get the current and desired depth values
    2. Create an object of class ImageTransport to read and convert between encodings
  */

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber depth_sub_;
  image_transport::Subscriber desired_depth_sub_;

  ImageConverter ic(width,height);

  depth_sub_ = it.subscribe("/camera/depth", 1,
  &ImageConverter::imageCb,&ic);
  desired_depth_sub_ = it.subscribe("/desired_camera/depth", 1,
  &ImageConverter::desired_imageCb,&ic);
  std::cout<<"created image subscriber"<<std::endl;

  /*
  1. creating Visual feature matrix
  2. setting up the servo task with EYEINHAND_CAMERA and USER_DEFINED INTERACTION MATRIX
  */

	std::vector <vpFeaturePoint> p(total_features), pd(total_features);
  vpServo task;
  vpMatrix L = ic.get_Interaction_Matrix();
	
	for (unsigned int i=0;i<p.size();i++)
	{
		task.addFeature(p[i], pd[i]);
	}
	
	task.setServo( vpServo::EYEINHAND_CAMERA);
	task.setInteractionMatrixType(vpServo::USER_DEFINED);
  
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

/*
  1. Read the desired depth points from the ImageTransport class 
  2. set the desired feature points

*/


  vpImage< unsigned char> Desired_I;
  cv::Mat desired_depth;
  cv::Mat desired_im;
  // std::cout<<"1"<<std::endl;
  // while(desired_depth.empty())
  // {
  //   desired_depth = ic.get_desired_depth();
  //   desired_im = ic.get_desired_image();
  // }
  
  // std::cout<<desired_depth<<std::endl;
  // vpImageConvert::convert(desired_im,Desired_I);

  // int check = 0;
  // try
  // {
  //   std::cout<<"P desired assigning..........."<<std::endl;
  //   for (unsigned int i=0;i<height;i++)
  //   {
  //     for(unsigned int j=0;j<width;j++)
  //     {
  //       pd[check].set_x(i);
  //       pd[check].set_y(j);
  //       pd[check].set_Z(desired_depth.at<float>(i,j));
  //       check++;
  //     }
  //   } 
  // }
  // catch (const vpException &e)
  // {
  //   std::cout<< " pd initialize error";
  //   return EXIT_FAILURE;
  // }
  // std::cout<< "pd initialize over ........"<<std::endl;
    

  /*
  Code to plot
  */

  // vpPlot *plotter = nullptr;

  // if ( opt_plot )
  // {
  //   plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
  //                         "Real time curves plotter" );
  //   plotter->setTitle( 0, "Visual features error" );
  //   plotter->setTitle( 1, "Camera velocities" );
  //   plotter->initGraph( 0, 8 );
  //   plotter->initGraph( 1, 6 );
  //   }Desired


  /*
  1. Declaring the cv::Mat files for the current images
  */

  vpImage< unsigned char> I;
  cv::Mat im;
  cv::Mat depth;
  //  std::cout<<"2"<<std::endl;
  // while(depth.empty())
  // {
  //   im = ic.get_current_image();
  //   depth = ic.get_current_depth();
  // }

  bool final_quit = false;
  bool has_converged = false;
  bool servo_started = false;
  double sim_time            = 0;
  double sim_time_prev       = sim_time;
  double sim_time_init_servo = sim_time;
  double sim_time_img        = sim_time;
  double wait_time = 0.02;

  //  std::cout<<"3"<<std::endl;
	while( !final_quit)
	{
    std::cout<<"enterring while loop"<<std::endl;
    sim_time = sim_time + wait_time;

    /*
    1. Display current image
    
    */
    try
    {
      vpImageConvert::convert(im,I);
      std::cout<<"converted image"<<std::endl;
      g.acquire(J);
      // cv::imshow("New Window",im);
		  // vpDisplay::display(J);
    }
		catch(const vpException &e)
    {
      std::cout<<"image display error"<<std::endl;
      return EXIT_FAILURE;
    }

    /*
      1. Read the current depth points from the ImageTransport class 
      2. set the current feature points
      3. Calculate the Interaction Matrix

    */

    im = ic.get_current_image();
    depth = ic.get_current_depth();
    L = ic.get_Interaction_Matrix();
    int check = 0;
    std::cout<<"p assign---------------------"<<std::endl;

    for (unsigned int i=0;i<height;i++)
    {
      for(unsigned int j=0;j<width;j++)
      {
        p[check].set_x(i);
        p[check].set_y(j);
        p[check].set_Z(depth.at<float>(i,j));
        // std::cout<<p[check].get_Z()<<std::endl;
        check++;
      }
    }
    std::cout<<"updated feature list"<<std::endl;

    /*
    Computing the control law
    */
    vpColVector v_c( 6 );
    task.L = L;
    std::cout<<L<<std::endl;
    std::cout<<"created vector"<<std::endl;
    std::cout<<"task dimension "<< task.getDimension()<<std::endl;
    v_c = task.computeControlLaw();
    std::cout<<"computed Control Law"<<std::endl;
    std::cout<<"task error"<<task.computeError()<<std::endl;
      
    std::cout<<"------------------------------------"<<std::endl;

    vpMatrix interac=task.getInteractionMatrix();
    vpServo::vpServoPrintType disp = vpServo::ALL;
    task.print(disp);
    std::cout<< "interaction matrix : "<<interac.getRow(0)<<std::endl;
    std::cout<<"computed Control Law"<<std::endl;
        
    // vpServoDisplay::display( task, cam, J );

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

    /*
    1. Display Error and publish it
    */
    double error = task.getError().sumSquare();
    std::stringstream ss;
    ss << "||error||: " << error;
    // vpDisplay::displayText( J, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
    
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
      // vpDisplay::displayText( J, 100, 20, "Servo task has converged", vpColor::red );
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
    if ( vpDisplay::getClick( J, button, false ) )
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

    // vpDisplay::flush( J );
    // vpDisplay::flush( Desired_J );
    // vpDisplay::flush(Desired);
    // Slow down the loop to simulate a camera at 50 Hz
    } // end while                        
    if ( !final_quit )
    {
    while ( !final_quit )
    {
        im = ic.get_current_image();
        vpImageConvert::convert(im,I);
        g.acquire(J);
        // vpDisplay::display( J);

        // vpDisplay::displayText( J, 20, 20, "Click to quit the program.", vpColor::red );
        // vpDisplay::displayText( J, 40, 20, "Visual servo converged.", vpColor::red );

        if ( vpDisplay::getClick( J, false ) )
        {
        final_quit = true;
        }

        // vpDisplay::flush(J );
        // vpDisplay::flush( Desired_J);
    }
    }
  // std::cout<<"4"<<std::endl;
  spinner.spin(); // spin() will not return until the node has been shutdown
  }
  
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
