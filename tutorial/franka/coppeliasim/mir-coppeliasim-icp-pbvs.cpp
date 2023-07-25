
#include <cassert>
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
#include "geometry_msgs/Pose.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>

static const std::string OPENCV_WINDOW = "Image window";


//!--******************************************************************************************************

class PointCloud
{

  // Eigen::Matrix4f homogenous_mat;
  // cv::Mat mat;
  
  vpHomogeneousMatrix mat;
  

public:
  PointCloud()
  {
 std::cout<<"class started"<<std::endl;
  }

  ~PointCloud()
  {

  }
void icp_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  // std::cout<< "subscriber called"<<'\n';
  double x = msg->position.x;
  double y = msg->position.y;
  double z = msg->position.z;
  vpTranslationVector trans(x,y,z);
  // vpTranslationVector trans(1.0,2.0,3.0);

  float W = msg->orientation.w;
  float X = msg->orientation.x;
  float Y = msg->orientation.y;
  float Z = msg->orientation.z;

  vpQuaternionVector quat(X,Y,Z,W);

  // std::cout<<"quaternion"<<quat<<std::endl;
  // std::cout<<"position"<<trans<<std::endl;
  // std::cout<<"------------------------------------------------------------------------"<<std::endl;

  // vpHomogeneousMatrix mat1(trans, quat);

  // mat(mat1);

  mat.buildFrom(trans,quat);

  // std::cout<<mat<<std::endl;
  // Eigen::Matrix3f mat3 = Eigen::Quaternionf(W, X, Y, Z).toRotationMatrix();
  // homogenous_mat = Eigen::Matrix4f::Identity();
  // homogenous_mat.block(0,0,3,3) << mat3;
  // homogenous_mat(0,3) = position.at(0);
  // homogenous_mat(1,3) = position.at(1);
  // homogenous_mat(2,3) = position.at(2);



//     for (size_t i = 0; i < 4; i++)
//   {
//     for (size_t j = 0; j < 4; j++)
//     {
//       mat.at<float>(i,j) = homogenous_mat(i,j);
//     }
// }
}

vpHomogeneousMatrix get_matrix()
{
  // std::cout<<"from the return function : "<<mat<<std::endl;
  return mat;
}


};


//!--******************************************************************************************************
//----------------------------------------------------------------

int
main( int argc, char **argv )
{

  //!--******************************************************************************************************
  int opt_quad_decimate          = 2;
  bool opt_verbose               = true;
  bool opt_plot                  = true;
  bool opt_adaptive_gain         = true;
  bool opt_task_sequencing       = false;
  double convergence_threshold   = 0.0005;
    double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad( 0.5 );
//!--******************************************************************************************************
try
{

  /*
   1. Create ROS node
   2. Create Velocity Publisher
   3. Create Error Publisher
  */
//!--******************************************************************************************************
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

//!--******************************************************************************************************
  /*
  
  1. Create ROS Grabber and Camera parameter object
  2. Read the current and desired rgb images from their ROS topics
  3. Read the camera parameters of both the current and desired cameras.
  */
//!--******************************************************************************************************
  vpDisplay::vpScaleType scale = vpDisplay::SCALE_DEFAULT;
  vpImage< unsigned char> J;
	vpROSGrabber g;
  vpCameraParameters cam;

  try
  {
    g.setImageTopic("/realsense/color/image_raw");
    g.setCameraInfoTopic("/realsense/color/camera_info");
    g.open(argc,argv);
    g.acquire(J);
    g.getCameraInfo( cam );


    // h.setImageTopic("/desired_camera/image/");
    // h.setCameraInfoTopic("/desired_camera/depth/camera_info");
    // h.open(argc,argv);
    // h.acquire(Desired_J);
    // h.getCameraInfo( desired_cam );
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

  const std::vector<int> &_sz{height,width};

  // vpDisplayOpenCV dc; 
  // try
  // {
  //   dc.init(J);
  //   vpDisplay::setWindowPosition(J,400,100);
  //   vpDisplay::setTitle(J,"Current Depth image");
  //   vpDisplay::display(J);
  //   // vpDisplayOpenCV dd( Desired_J, Desired_I.getWidth(), Desired_I.getHeight(), "Desired Depth image");

  // }
  // catch( const vpException &e )
  // {
  //   std::cout << "Error i opencv display " << e.what() << std::endl;
  //   std::cout << "Stop the robot " << std::endl;
  //   return EXIT_FAILURE;
  // }
//!--******************************************************************************************************

  /*
    1. Create subscriber to get the current and desired depth values
    2. Create an object of class ImageTransport to read and convert between encodings
  */
//!--******************************************************************************************************
  ros::NodeHandle nh;


  PointCloud pcd;
  Eigen::Matrix4f source;
  ros::Subscriber icp_sub;
  icp_sub = nh.subscribe("/icp/transform",1000,&PointCloud::icp_callback,&pcd);
  std::cout<<"subscriber launched"<<std::endl;
//!--******************************************************************************************************
  // ImageConverter ic(width,height);

  // depth_sub_ = it.subscribe("/camera/depth", 1,
  // &ImageConverter::imageCb,&ic);
  // desired_depth_sub_ = it.subscribe("/desired_camera/depth", 1,
  // &ImageConverter::desired_imageCb,&ic);
  // std::cout<<"created image subscriber"<<std::endl;

  /*
  1. creating Visual feature matrix
  2. setting up the servo task with EYEINHAND_CAMERA and USER_DEFINED INTERACTION MATRIX
  */
//!--******************************************************************************************************

  // Creation of an homogeneous matrix that represent the displacement
  // the camera has to achieve to move from the desired camera frame
  // and the current one
  vpHomogeneousMatrix cdMc, cMo, oMo;
  // ... cdMc is here the result of a pose estimation


  // Desired pose used to compute the desired features
  vpHomogeneousMatrix M_target( vpTranslationVector( 0, 0.0, 1),
                              vpRotationMatrix( { 1, 0, 0, 0, 1, 0, 0, 0, 1 } ) );

  // cdMc = M_target;


  // Creation of the current visual feature s = (c*_t_c, ThetaU)
  vpFeatureTranslation s_t( vpFeatureTranslation::cdMc );
  vpFeatureThetaU s_tu( vpFeatureThetaU::cdRc );

  // Set the initial values of the current visual feature s = (c*_t_c, ThetaU)
  s_t.buildFrom( cdMc );
  s_tu.buildFrom( cdMc );


  // Build the desired visual feature s* = (0,0)
  vpFeatureTranslation s_star_t(vpFeatureTranslation::cdMc); // Default initialization to zero 
  vpFeatureThetaU s_star_tu(vpFeatureThetaU::cdRc);// Default initialization to zero

  // s_star_t.buildFrom( M_target );
  // s_star_t.buildFrom( M_target );

  vpColVector v; // Camera velocity
  double error;  // Task error

  // Creation of the visual servo task.
  vpServo task;

  // Visual servo task initialization
  // - Camera is monted on the robot end-effector and velocities are
  //   computed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA); 

  // - Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT); 

  // - Set the contant gain
      if ( opt_adaptive_gain )
    {
      std::cout << "Enable adaptive gain" << std::endl;
      vpAdaptiveGain lambda( 4, 2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
      task.setLambda( lambda );
    }
    else
    {
      task.setLambda( 1.2 );
    }

  // - Add current and desired translation feature
  task.addFeature(s_t, s_star_t); 
  // - Add current and desired ThetaU feature for the rotation
  task.addFeature(s_tu, s_star_tu); 


    // vpPlot *plotter = nullptr;
    // if ( opt_plot )
    // {
    //   plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( J.getWidth() ) + 80, 10,
    //                         "Real time curves plotter" );
    //   plotter->setTitle( 0, "Visual features error" );
    //   plotter->setTitle( 1, "Camera velocities" );
    //   plotter->initGraph( 0, 6 );
    //   plotter->initGraph( 1, 6 );
    //   plotter->setLegend( 0, 0, "error_feat_tx" );
    //   plotter->setLegend( 0, 1, "error_feat_ty" );
    //   plotter->setLegend( 0, 2, "error_feat_tz" );
    //   plotter->setLegend( 0, 3, "error_feat_theta_ux" );
    //   plotter->setLegend( 0, 4, "error_feat_theta_uy" );
    //   plotter->setLegend( 0, 5, "error_feat_theta_uz" );
    //   plotter->setLegend( 1, 0, "vc_x" );
    //   plotter->setLegend( 1, 1, "vc_y" );
    //   plotter->setLegend( 1, 2, "vc_z" );
    //   plotter->setLegend( 1, 3, "wc_x" );
    //   plotter->setLegend( 1, 4, "wc_y" );
    //   plotter->setLegend( 1, 5, "wc_z" );
    // }


    bool final_quit                           = false;
    bool has_converged                        = false;
    // bool send_velocities                      = false;
    bool servo_started                        = false;

    double sim_time            = 0;
    double sim_time_prev       = sim_time;
    double sim_time_init_servo = sim_time;
    double sim_time_img        = sim_time;
    double wait_time = 0.02;


  // Visual servoing loop. The objective is here to update the visual
  // features s = (c*_t_c, ThetaU), compute the control law and apply
  // it to the robot

  do {

    
    g.acquire( J, sim_time_img );
    // vpDisplay::display( J );
    
    cdMc = pcd.get_matrix();
    // Update the current visual feature s
    s_t.buildFrom(cdMc);  // Update translation visual feature
    s_tu.buildFrom(cdMc); // Update ThetaU visual feature

    vpColVector v_c( 6 );
    v_c = task.computeControlLaw(); // Compute camera velocity skew
    error =  ( task.getError() ).sumSquare(); // error = s^2 - s_star^2

    if ( opt_verbose )
    {
      std::cout<<" M current"<< cdMc <<std::endl;
      std::cout << "v_c: " << v_c.t() << std::endl;
      std::cout<< "error: " << error << std::endl;
    }  

    
    std_msgs::Float64 error_msg;
    error_msg.data = error;
    m_pub_feature_error.publish(error_msg);
    std::cout<<" error_published"<<std::endl;

    // std::cout<<"s_t" <<s_t.get_s()<<std::endl;
    // std::cout<<"s_star_t"<<s_star_t.get_s()<<std::endl;
    // std::stringstream ss;
    // ss << "error_t: " << error;
    // vpDisplay::displayText( J, 40, static_cast< int >(J.getWidth() ) - 150, ss.str(), vpColor::red );

  
    {
      // std::cout << "v_c: " << v_c.t() << std::endl;
      geometry_msgs::Twist vel_msg;
      vel_msg.linear.x  = v_c[0];
      vel_msg.linear.y  = v_c[1];
      vel_msg.linear.z  = v_c[2];
      vel_msg.angular.x = v_c[3];
      vel_msg.angular.y = v_c[4];
      vel_msg.angular.z = v_c[5];
      m_pub_end_effector_vel.publish( vel_msg );

      }

    //   vpMouseButton::vpMouseButtonType button;
    //   if ( vpDisplay::getClick( J, button, false ) )
    //   {
    //     switch ( button )
    //     {
    //     case vpMouseButton::button1:
    //       // send_velocities = !send_velocities;
    //       break;

    //     case vpMouseButton::button3:
    //       final_quit = true;
    //       v_c        = 0;
    //       break;

    //     default:
    //       break;
    //     }
    //   }
    // vpDisplay::flush( J );


  } while ( !final_quit ); // Stop the task when current and desired visual features are close



    // while ( !final_quit )
    // {
    //   sim_time = sim_time + wait_time;
      
    //   vpHomogeneousMatrix cMo_vec;

    //   cMo_vec = pcd.get_matrix();

    //   std::cout<<"M_ICP(P)"<<cMo_vec<<std::endl;

    //   std::cout<<"P_desired"<<cdMc<<std::endl;

    //   vpColVector v_c( 6 );
    //   v_c = task.computeControlLaw();

    //   if ( opt_verbose )
    //     {
    //       std::cout << "v_c: " << v_c.t() << std::endl;
    //     }      

    //   vpTranslationVector cd_t_c = cdMc.getTranslationVector();
    //   vpThetaUVector cd_tu_c     = cdMc.getThetaUVector();
    //   double error_tr            = sqrt( cd_t_c.sumSquare() );
    //   double error_tu            = vpMath::deg( sqrt( cd_tu_c.sumSquare() ) );
    
    // std::stringstream ss;
    // ss << "error_t: " << error_tr;
    // vpDisplay::displayText( J, 20, static_cast< int >( J.getWidth() ) - 150, ss.str(), vpColor::red );
    // ss.str( "" );
    // ss << "error_tu: " << error_tu;
    // vpDisplay::displayText( J, 40, static_cast< int >(J.getWidth() ) - 150, ss.str(), vpColor::red );

    //     std_msgs::Float64 error_msg;
    //     error_msg.data = error_tr;
    //     m_pub_feature_error.publish(error_msg);
    //     // std::cout<<" error_published"<<std::endl;

    //     if ( opt_verbose )
    //       std::cout << "ercror translation: " << error_tr << " ; error rotation: " << error_tu << std::endl;

    //     if ( !has_converged && error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
    //     {
    //       has_converged = true;
    //       std::cout << "Servo task has converged"
    //                 << "\n";
    //       vpDisplay::displayText( J, 100, 20, "Servo task has converged", vpColor::red );
    //     }

    //     {
    //       // std::cout << "v_c: " << v_c.t() << std::endl;
    //       geometry_msgs::Twist vel_msg;
    //       vel_msg.linear.x  = v_c[0];
    //       vel_msg.linear.y  = v_c[1];
    //       vel_msg.linear.z  = v_c[2];
    //       vel_msg.angular.x = v_c[3];
    //       vel_msg.angular.y = v_c[4];
    //       vel_msg.angular.z = v_c[5];
    //       m_pub_end_effector_vel.publish( vel_msg );

    //   }

    //   vpMouseButton::vpMouseButtonType button;
    //   if ( vpDisplay::getClick( J, button, false ) )
    //   {
    //     switch ( button )
    //     {
    //     case vpMouseButton::button1:
    //       // send_velocities = !send_velocities;
    //       break;

    //     case vpMouseButton::button3:
    //       final_quit = true;
    //       v_c        = 0;
    //       break;

    //     default:
    //       break;
    //     }
    //   }

    //   vpDisplay::flush( J );
    //  //Slow down the loop to simulate a camera at 50 Hz
    // } // end while                        
    // if ( !final_quit )
    // {
    //   while ( !final_quit )
    //   {
    //     g.acquire( J );
    //     vpDisplay::display( J );

    //     vpDisplay::displayText( J, 20, 20, "Click to quit the program.", vpColor::red );
    //     vpDisplay::displayText( J, 40, 20, "Visual servo converged.", vpColor::red );

    //     if ( vpDisplay::getClick( J, false ) )
    //     {
    //       final_quit = true;
    //     }

    //     vpDisplay::flush( J );
    //   }
    // }
    // // if ( traj_corners )
    // // {
    // //   delete[] traj_corners;
    // // }
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}


