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

#include <visp3/visual_features/vpFeaturePoint.h>

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    if ( traj_vip[i].size() )
    {
      // Add the point only if distance with the previous > 1 pixel
      if ( vpImagePoint::distance( vip[i], traj_vip[i].back() ) > 1. )
      {
        traj_vip[i].push_back( vip[i] );
      }
    }
    else
    {
      traj_vip[i].push_back( vip[i] );
    }
  }
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    for ( size_t j = 1; j < traj_vip[i].size(); j++ )
    {
      vpDisplay::displayLine( I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2 );
    }
  }
}

int
main( int argc, char **argv )
{

    double opt_tagSize             = 0.5;
  bool display_tag               = true;
  int opt_quad_decimate          = 2;
  bool opt_verbose               = true;
  bool opt_plot                  = true;
  bool opt_adaptive_gain         = true;
  bool opt_task_sequencing       = false;
  double convergence_threshold   = 0.0005;
  bool opt_coppeliasim_sync_mode = false;
  try
  {
    ros::init( argc, argv, "visp_ros" );
    ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
    ros::Rate loop_rate( 1000 );
    ros::spinOnce();

    std::string m_topic_end_effector_vel;
    m_topic_end_effector_vel = "/fakeFCI/end_effector_vel";

    ros::NodeHandlePtr m = boost::make_shared< ros::NodeHandle >();
    ros::Publisher m_pub_end_effector_vel;
    m_pub_end_effector_vel = m->advertise<geometry_msgs::Twist >(m_topic_end_effector_vel, 1);


    std::string m_topic_feature_error;
    m_topic_feature_error = "/feature_error";

    ros::NodeHandlePtr l = boost::make_shared< ros::NodeHandle >();
    ros::Publisher m_pub_feature_error;
    m_pub_feature_error = l->advertise<std_msgs::Float64 >(m_topic_feature_error, 1);

    vpImage< unsigned char > I;
    vpROSGrabber g;
    g.setImageTopic( "/camera/color/image_raw" );
    g.setCameraInfoTopic( "/camera/color/camera_info" );
    // g.setImageTopic( "/coppeliasim/franka/camera/image" );
    // g.setCameraInfoTopic( "/coppeliasim/franka/camera/camera_info" );
    


    g.open( argc, argv );
    ros::Publisher m_pub_robotStateCmd;
    g.acquire( I );

    std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
    vpCameraParameters cam;

    g.getCameraInfo( cam );
    std::cout << cam << std::endl;
    vpDisplayOpenCV dc( I, 10, 10, "Color image" );

    vpDetectorAprilTag::vpAprilTagFamily tagFamily                  = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector( tagFamily );
    detector.setAprilTagPoseEstimationMethod( poseEstimationMethod );
    detector.setDisplayTag( display_tag );
    detector.setAprilTagQuadDecimate( opt_quad_decimate );

    // Servo
    vpHomogeneousMatrix cMo, oMo;

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo( vpTranslationVector( 0.0, 0.0, opt_tagSize * 3 ),
                              vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );
    
    // Create visual features
    std::vector< vpFeaturePoint > p( 4 ), pd( 4 ); // We use 4 points

    // Define 4 3D points corresponding to the CAD model of the Apriltag
    std::vector< vpPoint > point( 4 );
    point[0].setWorldCoordinates( -opt_tagSize / 2., -opt_tagSize / 2., 0 );
    point[1].setWorldCoordinates( opt_tagSize / 2., -opt_tagSize / 2., 0 );
    point[2].setWorldCoordinates( opt_tagSize / 2., opt_tagSize / 2., 0 );
    point[3].setWorldCoordinates( -opt_tagSize / 2., opt_tagSize / 2., 0 );



    vpServo task;
    // Add the 4 visual feature points
    for ( size_t i = 0; i < p.size(); i++ )
    {
      task.addFeature( p[i], pd[i] );
    }
    task.setServo( vpServo::EYEINHAND_CAMERA );
    task.setInteractionMatrixType( vpServo::CURRENT );

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

    vpPlot *plotter = nullptr;

    if ( opt_plot )
    {
      plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
                            "Real time curves plotter" );
      plotter->setTitle( 0, "Visual features error" );
      plotter->setTitle( 1, "Camera velocities" );
      plotter->initGraph( 0, 8 );
      plotter->initGraph( 1, 6 );
      plotter->setLegend( 0, 0, "error_feat_p1_x" );
      plotter->setLegend( 0, 1, "error_feat_p1_y" );
      plotter->setLegend( 0, 2, "error_feat_p2_x" );
      plotter->setLegend( 0, 3, "error_feat_p2_y" );
      plotter->setLegend( 0, 4, "error_feat_p3_x" );
      plotter->setLegend( 0, 5, "error_feat_p3_y" );
      plotter->setLegend( 0, 6, "error_feat_p4_x" );
      plotter->setLegend( 0, 7, "error_feat_p4_y" );
      plotter->setLegend( 1, 0, "vc_x" );
      plotter->setLegend( 1, 1, "vc_y" );
      plotter->setLegend( 1, 2, "vc_z" );
      plotter->setLegend( 1, 3, "wc_x" );
      plotter->setLegend( 1, 4, "wc_y" );
      plotter->setLegend( 1, 5, "wc_z" );
    }

    bool final_quit                           = false;
    bool has_converged                        = false;
    //bool send_velocities                      = false;
    bool servo_started                        = false;
    std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory


    // if ( 0 )
    // {
    //   // Instead of setting eMc from /coppeliasim/franka/eMc topic, we can set its value to introduce noise for example
    //   vpHomogeneousMatrix eMc;
    //   eMc.buildFrom( 0.05, -0.05, 0, 0, 0, M_PI_4 );
    //   robot.set_eMc( eMc );
    // }
    // std::cout << "eMc:\n" << robot.get_eMc() << std::endl;

    // robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
    // robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

    double sim_time            = 0;
    double sim_time_prev       = sim_time;
    double sim_time_init_servo = sim_time;
    double sim_time_img        = sim_time;
    double wait_time = 0.02;

    // vpHomogeneousMatrix wMc, wMo;
    // vpSimulatorCamera robot;
    // robot.setSamplingTime(0.040);
    // robot.getPosition(wMc);
    // unsigned int iter = 0;

    while ( !final_quit )
    {
      sim_time = sim_time + wait_time;
      g.acquire( I, sim_time_img );
      vpDisplay::display( I );

      std::vector< vpHomogeneousMatrix > cMo_vec;
      detector.detect( I, opt_tagSize, cam, cMo_vec );

      // {
      //   std::stringstream ss;
      //   ss << "Left click to " << ( send_velocities ? "stop the robot" : "servo the robot" )
      //      << ", right click to quit.";
      //   vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
      // }

      vpColVector v_c( 6 );

      // Only one tag is detected
      if ( cMo_vec.size() == 1 )
      {
        cMo = cMo_vec[0];
        //robot.getPosition(wMc);
        static bool first_time = true;
        if ( first_time )
        {
          // Introduce security wrt tag positionning in order to avoid PI rotation
          std::vector< vpHomogeneousMatrix > v_oMo( 2 ), v_cdMc( 2 );
          v_oMo[1].buildFrom( 0, 0, 0, 0, 0, M_PI );
          for ( size_t i = 0; i < 2; i++ )
          {
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
          }
          if ( std::fabs( v_cdMc[0].getThetaUVector().getTheta() ) <
               std::fabs( v_cdMc[1].getThetaUVector().getTheta() ) )
          {
            oMo = v_oMo[0];
          }
          else
          {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            oMo = v_oMo[1]; // Introduce PI rotation
          }

          // Compute the desired position of the features from the desired pose
          std::cout << "Initializing desired pose" << std::endl;
          for ( size_t i = 0; i < point.size(); i++ )
          {
            vpColVector cP, p_;
            point[i].changeFrame( cdMo * oMo, cP );
            point[i].projection( cP, p_ );

            pd[i].set_x( p_[0] );
            pd[i].set_y( p_[1] );
            pd[i].set_Z( cP[2] );
          }
          std::cout<<"end first time"<<std::endl;
        } // end first_time

      // Get tag corners
        std::vector< vpImagePoint > corners = detector.getPolygon( 0 );

        std::cout<< "updating tag corners"<<std::endl;
        // Update visual features
        for ( size_t i = 0; i < corners.size(); i++ )
        {
          // Update the point feature from the tag corners location
          vpFeatureBuilder::create( p[i], cam, corners[i] );
          // Set the feature Z coordinate from the pose
          vpColVector cP;
          point[i].changeFrame( cMo, cP );

          p[i].set_Z( cP[2] );
        }
        // Update visual features
        v_c = task.computeControlLaw();
        // v_c = task.computeControlLaw(iter * robot.getSamplingTime());

         vpServoDisplay::display( task, cam, I );
        for ( size_t i = 0; i < corners.size(); i++ )
        {
          std::stringstream ss;
          ss << i;
          // Display current point indexes
          vpDisplay::displayText( I, corners[i] + vpImagePoint( 15, 15 ), ss.str(), vpColor::red );
          // Display desired point indexes
          vpImagePoint ip;
          vpMeterPixelConversion::convertPoint( cam, pd[i].get_x(), pd[i].get_y(), ip );
          vpDisplay::displayText( I, ip + vpImagePoint( 15, 15 ), ss.str(), vpColor::red );
        }

        if ( first_time )
        {
          traj_corners = new std::vector< vpImagePoint >[corners.size()];
        }

        // Display the trajectory of the points used as features
        display_point_trajectory( I, corners, traj_corners );

        if ( opt_plot )
        {
          plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
          plotter->plot( 1, static_cast< double >( sim_time ), v_c );
        }

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
        }

        if ( first_time )
        {
          first_time = false;
        }
      } // end if (cMo_vec.size() == 1)
      else
      {
        v_c = 0; // Stop the robot
      }

      // if ( !send_velocities )
      // {
      //   v_c = 0; // Stop the robot
      // }

      //robot.setVelocity( vpRobot::CAMERA_FRAME, v_c );

      // if ( !send_velocities )
      // {
      //   v_c = 0; // Stop the robot
      // }

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
     // Slow down the loop to simulate a camera at 50 Hz
    } // end while                        
    if ( !final_quit )
    {
      while ( !final_quit )
      {
        g.acquire( I );
        vpDisplay::display( I );

        vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
        vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

        if ( vpDisplay::getClick( I, false ) )
        {
          final_quit = true;
        }

        vpDisplay::flush( I );
      }
    }
    if ( traj_corners )
    {
      delete[] traj_corners;
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