// Copyright 2008 Isis Innovation Limited
#include "ptam/System.h"
#include "ptam/OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/Tracker.h"
//#include "ptam/ARDriver.h"
#include "ptam/MapViewer.h"
#include "ptam/LevelHelpers.h"
#include "ptam/MapPoint.h"

#include <ptam_com/ptam_info.h>
#include <opencv/cv.h>
#include <cvd/vision.h>

#include "ptam/Navdata.h" //enddl22

using namespace CVD;
using namespace std;
using namespace GVars3;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Warning! These are the global valuables.

std::vector<float> z_sonar_measure;
std::vector<float> z_ptam_measure;
std::vector<float> x_norm,y_norm;
//const int nSamples=50;

const int nSamples=10;

System::System() :
  nh_("vslam"), image_nh_(""), first_frame_(true),mbPickPose(false),mbScale(false)
{

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
  pub_info_ = nh_.advertise<ptam_com::ptam_info> ("info", 1);
  pub_vslamStates_ = nh_.advertise<std_msgs::String>("/vslam/states", 1);
  srvPC_ = nh_.advertiseService("pointcloud", &System::pointcloudservice,this);
  srvKF_ = nh_.advertiseService("keyframes", &System::keyframesservice,this);

  sub_imu_ = nh_.subscribe("imu", 100, &System::imuCallback, this);
  sub_kb_input_ = nh_.subscribe("key_pressed", 100, &System::keyboardCallback, this);

  //enddl22[[
  nh_nav = ros::NodeHandle("");
  nav_sub_ = nh_nav.subscribe<ptam::Navdata> ("ardrone/navdata", 1, &System::navCallback, this);
  vslam_cmd_sub_ = nh_.subscribe("/vslam/cmd", 100, &System::cmdCallback, this);
  cout<<"This node name is "<<ros::this_node::getName().c_str()<<endl;
  //enddl22]]
  
  
  image_nh_.setCallbackQueue(&image_queue_);
  // get topic to subscribe to:
  std::string topic = image_nh_.resolveName("image");
  if (topic == "/image")
  {
    ROS_WARN("video source: image has not been remapped! Typical command-line usage:\n"
        "\t$ ./ptam image:=<image topic>");
  }

  image_transport::ImageTransport it(image_nh_);
  sub_image_ = it.subscribe(topic, 1, &System::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));
  pub_preview_image_ = it.advertise("vslam/preview", 1);
}

System::~System()
{

  #ifdef Z_FILE_SAVE
  for (int i=0;i<z_sonar_measure.size();i++)
  {
     file_sonar_z<<z_sonar_measure[i]<<endl;
     file_ptam_z<<z_ptam_measure[i]<<endl;
  }
  file_sonar_z.close();
  file_ptam_z.close();
  file_ptam_fixed_z.close();
  file_scale.close();
  #endif
}

void System::init(const CVD::ImageRef & size)
{
  img_bw_.resize(size);
  img_rgb_.resize(size);

  mpCamera = new ATANCamera("Camera");

  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(size, *mpCamera, *mpMap, *mpMapMaker);

  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);

  if(ParamsAccess::fixParams->gui){
    mGLWindow = new GLWindow2(size, "PTAM");
    mpMapViewer = new MapViewer(*mpMap, *mGLWindow);

    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
    GUI.ParseLine("DrawMap=0");
    GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  }

  
  z_ptam_measure.clear();
  z_sonar_measure.clear();
  x_norm.clear();
  y_norm.clear();

  #ifdef Z_FILE_SAVE
  
  file_sonar_z.open("sonar_z.txt");
  
  file_ptam_z.open("ptam_z.txt");
  
  file_ptam_fixed_z.open("ptam_z_fixed.txt");
  
  file_scale.open("scale.txt");
  #endif
  
  ardrone_scale.resize(2);

  // For normalisation.
  //sigma.resize(2);
  //mean.resize(2);
  
  last_z=0;
   
 
  
}


void System::Run()
{
  while(ros::ok()){

    ros::getGlobalCallbackQueue()->callAvailable();
    image_queue_.callAvailable(ros::WallDuration(0.01));
  }
}

void System::imageCallback(const sensor_msgs::ImageConstPtr & img)
{


  ROS_ASSERT(img->encoding == sensor_msgs::image_encodings::MONO8 && img->step == img->width);

  VarParams *varParams = ParamsAccess::varParams;

  if(first_frame_){
    init(CVD::ImageRef(img->width, img->height));
    first_frame_ = false;
  }

  TooN::SO3<double> imu_orientation;
  if (varParams->MotionModelSource == ptam::PtamParams_MM_IMU)
  {
    sensor_msgs::Imu imu;


    if (!findClosest(img->header.stamp, imu_msgs_, &imu, 0.01))
    {
      ROS_WARN("no imu match, skipping frame");
      return;
    }
    if (!transformQuaternion(img->header.frame_id, imu.header, imu.orientation, imu_orientation))
    {
      return;
    }
  }


//  -------------------
  // TODO: avoid copy
  CVD::BasicImage<CVD::byte> img_tmp((CVD::byte *)&img->data[0], CVD::ImageRef(img->width, img->height));
  CVD::copy(img_tmp, img_bw_);

  bool tracker_draw = false;

    static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
    bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;

  if(ParamsAccess::fixParams->gui){
    CVD::copy(img_tmp, img_rgb_);

    mGLWindow->SetupViewport();
    mGLWindow->SetupVideoOrtho();
    mGLWindow->SetupVideoRasterPosAndZoom();
    tracker_draw = !bDrawMap;
  }

  mpTracker->TrackFrame(img_bw_, tracker_draw, imu_orientation);

  publishPoseAndInfo(img->header);

  publishPreviewImage(img_bw_, img->header);
  std::cout << mpMapMaker->getMessageForUser();

  if(ParamsAccess::fixParams->gui){
    string sCaption;

    if (bDrawMap)
      mpMapViewer->DrawMap(mpTracker->GetCurrentPose());

    if (bDrawMap)
      sCaption = mpMapViewer->GetMessageForUser();
    else
      sCaption = mpTracker->GetMessageForUser();
    mGLWindow->DrawCaption(sCaption);
    mGLWindow->DrawMenus();
    mGLWindow->swap_buffers();
    mGLWindow->HandlePendingEvents();
  }

    if(mpTracker->getTrailTrackingComplete() && !mbPickPose)
    {
      try
      {
         cout<<"Pick up the initial pose"<<endl;
         tf_sub_.lookupTransform("/world","/imu",ros::Time(0),worldToimuTF);
         cout<<"x="<<worldToimuTF.getOrigin().x()<<" y="
         <<worldToimuTF.getOrigin().y()<<" z="<<worldToimuTF.getOrigin().z()<<endl;
         mbPickPose=true;
         
         #ifdef Z_FILE_SAVE
         file_ptam_fixed_z<<worldToimuTF.getOrigin().z()<<endl;
         #endif
 
         //se3InitPose.
      }
      catch (tf::TransformException ex)
      {
         ROS_ERROR("%s",ex.what());
      }
  }
  if(mpTracker->getTrailTrackingComplete() && mbPickPose)
  {
    std_msgs::String str;
    ros::Time now = ros::Time::now();
    tf::Transform transform;
    transform.setOrigin(worldToimuTF.getOrigin());
    transform.setRotation(worldToimuTF.getRotation());
    //btQuaternion q(0,0,0,0);
    //transform.setRotation(tf::Quaternion(0,0,0));
    tf::StampedTransform tf(transform, now, "/world", "/ptam_fixed");
    tf_pub_.sendTransform(tf);
    
    str.data = "PTAM_initialized";
    pub_vslamStates_.publish(str);

    ParamsAccess Params;
    ptam::PtamParamsConfig* pPars = Params.varParams;
    pPars->AutoInit = 0;
    
  }
  else
  {
    std_msgs::String str;
    str.data = "PTAM_uninitialized";
    pub_vslamStates_.publish(str);
  }
    
    
}


void System::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  imu_msgs_.push(*msg);
  if (imu_msgs_.size() > 100)
    imu_msgs_.pop();
}


void System::cmdCallback(const std_msgs::String & msg)
{
  if(msg.data=="ptam_initial")
  {
    ParamsAccess Params;
    cout<<"Set auto init"<<endl;
    ptam::PtamParamsConfig* pPars = Params.varParams;
    pPars->AutoInit = 1;
  }  
}

template<class T>
  bool System::findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay)
  {
    double best_dt(1e9);
    double tmp_dt;
    //  size_t qs_before = queue.size();
    //  int i = 0;
    while (!queue.empty())
    {
      const T & curr_obj = queue.front();
      tmp_dt = (timestamp - curr_obj.header.stamp).toSec();

      if (tmp_dt < -max_delay)
        break;
      if (std::abs(tmp_dt) < best_dt)
      {
        best_dt = std::abs(tmp_dt);
        *obj = curr_obj;
        //      i++;
      }
      queue.pop();
    }
    if (best_dt > max_delay)
    {
      return false;
    }
    else
    {
      return true;
    };
  }


void System::keyboardCallback(const std_msgs::StringConstPtr & kb_input){
  mpTracker->command(kb_input->data);
}

//enddl22
void System::navCallback(const ptam::NavdataConstPtr& nav_msg){
  nav_=*nav_msg;
}


bool System::transformQuaternion(const std::string & target_frame, const std_msgs::Header & header,
                                 const geometry_msgs::Quaternion & _q_in, TooN::SO3<double> & r_out)
{
  geometry_msgs::QuaternionStamped q_in, q_out;
  q_in.header = header;
  q_in.quaternion = _q_in;
  try
  {
    tf_sub_.transformQuaternion(target_frame, q_in, q_out);
    quaternionToRotationMatrix(q_out.quaternion, r_out);
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform quaternion: "<<e.what());
    return false;
  }
  return true;
}

bool System::transformPoint(const std::string & target_frame, const std_msgs::Header & header,
                            const geometry_msgs::Point & _p_in, TooN::Vector<3> & _p_out)
{
  geometry_msgs::PointStamped p_in, p_out;
  p_in.header = header;
  p_in.point = _p_in;
  try
  {
    tf_sub_.transformPoint(target_frame, p_in, p_out);
    _p_out[0] = p_out.point.x;
    _p_out[1] = p_out.point.y;
    _p_out[2] = p_out.point.z;
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform point: "<<e.what());
    return false;
  }
  return true;
}



// The error function F(x):
void myFunction( const vector_double &x, const vector_double &y, vector_double &out_f)
{

	out_f.resize(1);
  
  std::vector<float> vy = z_sonar_measure;
  std::vector<float> vx = z_ptam_measure;

  
  int n=vx.size();
  
  Eigen::MatrixXf X(n,2);
  Eigen::MatrixXf theta(2,1);
  theta(0,0) = x[0];
  theta(1,0) = x[1];
  Eigen::MatrixXf Y(n,1);
  Eigen::MatrixXf temp(n,1); //nx1
  Eigen::MatrixXf temp1(n,1);
  Eigen::MatrixXf sqr(1,1);
  

  for(int i=0;i<n;i++)
  {
    X(i,0) = 1;
    X(i,1) = vx[i];
    Y(i,0) = vy[i];    
  }
  
  temp = X*theta-Y;  //nx1
  temp1=temp.transpose();        // 1xn
  sqr = temp1*temp;   // 1xn * nx1 = 1x1
  float a=sqr(0,0);
  out_f[0] = a/(2*n);
}

bool System::findScale()
{
  vector_double		optimal_x;
	vector_double		initial_x;
	vector_double		increments_x;
	vector_double		y;

	CLevenbergMarquardt::TResultInfo	info;
	CTicTac	tictac;

	initial_x.resize(2);
	initial_x[0] = -1.2;	// x
	initial_x[1] = 1;	// y

	//increments_x.resize(2, 0.0001 );
	//increments_x.resize(2, 0.0001 );
	increments_x.resize(2, 0.01 );

	double T;
	size_t  N = 1;

	tictac.Tic();
	for (size_t k=0;k<N;k++)
		CLevenbergMarquardt::execute(optimal_x, initial_x, myFunction, increments_x, y, info ,
		false, /* verbose=false */
		10000, /* max iter= 200*/
		1e-2, //5.5, //1e-2,  /* tau = 1e-3*/
		1e-9, /* e1 = 1e-8 */
		1e-9, /* e2 = 1e-8 */
		false  );

	T = tictac.Tac() / N;

	cout << "Iterations: " << info.iterations_executed <<  endl;
	cout << "Final sqr error: " << info.final_sqr_err << endl;

	cout << "Final optimized position: " << optimal_x << endl;
  file_scale<<optimal_x[0]<<","<<optimal_x[1]<<endl;

	cout << "Time: " << T*1e6 << " us" << endl;

	info.path.saveToTextFile("lm-path.txt");
	cout << "Path saved to 'lm-path.txt'" << endl;
	ardrone_scale = optimal_x;
	mbScale=true;
  return 1;
  
}





void System::publishPoseAndInfo(const std_msgs::Header & header)
{
  ParamsAccess Params;
  double scale = Params.varParams->Scale;
  static double last_time = 0;
  static float fps=0;

  if (scale <= 0)
  {
    ROS_WARN_STREAM("scale ("<<scale<<") <= 0, set to 1");
    scale = 1;
  }

  if (mpTracker->getTrackingQuality() && mpMap->IsGood())
  {
    //TooN::SE3<double> plane = mpMapMaker->getPlane().inverse();
    TooN::SE3<double> pose = mpTracker->GetCurrentPose().inverse();
    TooN::Matrix<3, 3, double> r = pose.get_rotation().get_matrix();
    TooN::Vector<3, double> & t = pose.get_translation();


    //rotx(pi/2)*rotz(pi/2)
    TooN::Matrix<3, 3, double> iRb;
    iRb(0,0)=0;
    iRb(1,0)=0;
    iRb(2,0)=1;
    iRb(0,1)=1;
    iRb(1,1)=0;
    iRb(2,1)=0;
    iRb(0,2)=0;
    iRb(1,2)=1;
    iRb(2,2)=0;
    r = iRb.T()*r;
    t = iRb.T()*t;
    
    
        
    
    if(mbPickPose)
    {

    	if(mbScale)
    	{
        ros::Time now = ros::Time::now();
    		tf::StampedTransform transform_test(tf::Transform(btMatrix3x3(r(0, 0), r(0, 1), r(0, 2), 
    		                                                       r(1, 0), r(1, 1), r(1, 2),
    		                                                       r(2, 0), r(2, 1), r(2, 2)), 
    		 btVector3(ardrone_scale[0]+ardrone_scale[1]*t[0], 
        		       ardrone_scale[0]+ardrone_scale[1]*t[1], 
    		           ardrone_scale[0]+ardrone_scale[1]*t[2] )), now, "/ptam_fixed", "/body");
    	    	tf_pub_.sendTransform(transform_test);

          try
          {
            tf_sub_.lookupTransform("/world","/body",ros::Time(0),worldToBodyTF);
          }
          catch(tf::TransformException ex)
          {
            ROS_ERROR("%s",ex.what());
    }

    {
      geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);
              const tf::Quaternion & q = worldToBodyTF.getRotation();
              const tf::Vector3 & t = worldToBodyTF.getOrigin();
      msg_pose->pose.pose.orientation.w = q.w();
      msg_pose->pose.pose.orientation.x = q.x();
      msg_pose->pose.pose.orientation.y = q.y();
      msg_pose->pose.pose.orientation.z = q.z();
      msg_pose->pose.pose.position.x = t.x();
      msg_pose->pose.pose.position.y = t.y();
      msg_pose->pose.pose.position.z = t.z();

      TooN::Matrix<6> covar = mpTracker->GetCurrentCov();
      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
        msg_pose->pose.covariance[i] = sqrt(fabs(covar[i % 6][i / 6]));

      msg_pose->pose.covariance[1] = mpTracker->GetCurrentKF().dSceneDepthMedian;

      msg_pose->header = header;
              msg_pose->header.stamp = now;

      pub_pose_.publish(msg_pose);
    }

    {
    	ptam_com::ptam_infoPtr msg_info(new ptam_com::ptam_info);
      double diff = header.stamp.toSec() - last_time;
      if (diff < 1.0 && diff > 0.005)
        fps = fps * 0.8 + 0.2 / diff;
      if (fabs(fps) > 200)
        fps = 200;
      last_time = header.stamp.toSec();

      msg_info->header = header;
      msg_info->fps = fps;
      msg_info->mapQuality = mpMap->bGood;
      msg_info->trackingQuality = mpTracker->getTrackingQuality();
      msg_info->trackerMessage = mpTracker->GetMessageForUser();
      msg_info->keyframes = mpMap->vpKeyFrames.size();
      pub_info_.publish(msg_info);
    }
    	}

      float cur_z = nav_.altd;
      if(1)
      if( abs( cur_z - last_z) > 0.005 && z_sonar_measure.size() < 100 )   // 7.5cm/s
      {
        z_sonar_measure.push_back( -(nav_.altd-worldToimuTF.getOrigin().z()));
        z_ptam_measure.push_back(t[2]);
      }
      else cout<<"Ignore the sample"<<endl;
      last_z = nav_.altd;
      
      if(z_sonar_measure.size() >=100) mbMaxSample=true;
      

      //Run LM optimizationHere
      if (z_ptam_measure.size()%nSamples==0 && !mbMaxSample)
      {
        cout<<"\n\nCollecting "<<z_ptam_measure.size()<<" number of data"<<endl;
        //normalisation();
        findScale();
      }

      // Do we need to keep all data or just keep the current number of samples for
      // optimisation?
        
      //vz_sonar_measure.clear();
      //vz_ptam_measure.clear();

      

      } //mbPickPose
// Inkyu]
  }
}



void System::normalisation()
{
  vector<float> x= z_ptam_measure;
  vector<float> y= z_sonar_measure; 
  sort(x.begin(), x.end());  // Sorting in ascending order.
  sort(y.begin(), y.end());
  float x_max,x_min,x_range;
  float y_max,y_min,y_range;
  x_min = x.front();         // Therefore, the first element is the lowest one.
  x_max = x.back();
  y_min = y.front();
  y_max = y.back();
  x_range = x_max - x_min;
  y_range = y_max - y_min;

  //cout<<"x range"<<x_range<<endl;
  
  int m = x.size();
  
  float sum_x = accumulate(x.begin(),x.end(),0);
  float sum_y = accumulate(y.begin(),y.end(),0);
  
  float mean_x = sum_x/m;
  float mean_y = sum_y/m;

  //cout<<"mean x = "<<mean_x<<endl;

  vector<float> temp_x,temp_y;
  for(int i=0;i<m;i++)
  {
    temp_x.push_back(pow(x[i]-mean_x,2));
    temp_y.push_back(pow(y[i]-mean_y,2));
  }
  
  float sum_xx=0;
  float sum_yy=0;
  for(int i=0;i<m;i++)
  {
     sum_xx+= temp_x[i];
     sum_yy+= temp_y[i];
  }

  
  for (int i=0;i<m;i++)
  {
    x_norm.push_back( (x[i] - mean_x)/x_range);
    y_norm.push_back( (y[i] - mean_y)/y_range);
  }

  //cout<<"x_norm " <<x_norm<<endl;
}

void System::publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header)
{
  CVD::Image<TooN::Vector<2> > & grid = mpTracker->ComputeGrid();
  std::list<Trail> & trails = mpTracker->getTrails();
  bool drawGrid = mpTracker->getTrailTrackingComplete();
  bool drawTrails = mpTracker->getTrailTrackingStarted();

  if (pub_preview_image_.getNumSubscribers() > 0)
  {
    CVD::ImageRef sub_size(img.size()/2);
    sensor_msgs::ImagePtr img_msg(new sensor_msgs::Image);
    img_msg->header = header;
    img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    img_msg->width = sub_size.x;
    img_msg->height = sub_size.y;
    img_msg->step = sub_size.x;
    img_msg->is_bigendian = 0;
    img_msg->data.resize(sub_size.x * sub_size.y);

    // subsample image
    CVD::BasicImage<CVD::byte> img_sub((CVD::byte *)&img_msg->data[0], sub_size);
    CVD::halfSample(img, img_sub);

    // set opencv pointer to image
    IplImage * ocv_img = cvCreateImageHeader(cvSize(img_sub.size().x, img_sub.size().y), IPL_DEPTH_8U, 1);
    ocv_img->imageData = (char*)&img_msg->data[0];

    int dim0 = grid.size().x;
    int dim1 = grid.size().y;

    if (drawGrid)
    {
      for (int i = 0; i < dim0; i++)
      {
        for (int j = 0; j < dim1 - 1; j++)
          cvLine( ocv_img, cvPoint(grid[i][j][0]/2, grid[i][j][1]/2), cvPoint(grid[i][j + 1][0]/2, grid[i][j + 1][1]/2),
                 CV_RGB(50, 50, 50)
          );

        for (int j = 0; j < dim1 - 1; j++)
          cvLine(ocv_img, cvPoint(grid[j][i][0]/2, grid[j][i][1]/2), cvPoint(grid[j + 1][i][0]/2, grid[j + 1][i][1]/2),
                 CV_RGB(50, 50, 50)
          );
      }
    }

    if (drawTrails)
    {

      ParamsAccess Params;
      int level = Params.fixParams->InitLevel;

      for (std::list<Trail>::iterator i = trails.begin(); i != trails.end(); i++)
      {
        cvLine(ocv_img, cvPoint(LevelZeroPos(i->irCurrentPos.x, level)/2, LevelZeroPos(i->irCurrentPos.y, level)/2),
               cvPoint(LevelZeroPos(i->irInitialPos.x, level)/2, LevelZeroPos(i->irInitialPos.y, level)/2),
               CV_RGB(0, 0, 0), 2);
      }
    }

    pub_preview_image_.publish(img_msg);
    cvReleaseImageHeader(&ocv_img);
  }
}

//Weiss{
bool System::pointcloudservice(ptam_com::PointCloudRequest & req, ptam_com::PointCloudResponse & resp)
{
	static unsigned int seq=0;
	int dimension   = 6;

	resp.pointcloud.header.seq=seq;
	seq++;
	resp.pointcloud.header.stamp = ros::Time::now();
	resp.pointcloud.height = 1;
	resp.pointcloud.header.frame_id = "/world";
	if(mpMap->bGood)
	{
		resp.pointcloud.width = mpMap->vpPoints.size();
		resp.pointcloud.fields.resize(dimension);
		resp.pointcloud.fields[0].name = "x";
		resp.pointcloud.fields[0].offset = 0*sizeof(uint32_t);
		resp.pointcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[0].count = 1;
		resp.pointcloud.fields[1].name = "y";
		resp.pointcloud.fields[1].offset = 1*sizeof(uint32_t);
		resp.pointcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[1].count = 1;
		resp.pointcloud.fields[2].name = "z";
		resp.pointcloud.fields[2].offset = 2*sizeof(uint32_t);
		resp.pointcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[2].count = 1;
		resp.pointcloud.fields[3].name = "rgb";
		resp.pointcloud.fields[3].offset = 3*sizeof(uint32_t);
		resp.pointcloud.fields[3].datatype = sensor_msgs::PointField::INT32;
		resp.pointcloud.fields[3].count = 1;
		resp.pointcloud.fields[4].name = "KF";
		resp.pointcloud.fields[4].offset = 4*sizeof(uint32_t);
		resp.pointcloud.fields[4].datatype = sensor_msgs::PointField::INT32;
		resp.pointcloud.fields[4].count = 1;
		resp.pointcloud.fields[5].name = "lvl";
		resp.pointcloud.fields[5].offset = 5*sizeof(uint32_t);
		resp.pointcloud.fields[5].datatype = sensor_msgs::PointField::INT32;
		resp.pointcloud.fields[5].count = 1;

		resp.pointcloud.point_step = dimension*sizeof(uint32_t);
		resp.pointcloud.row_step = resp.pointcloud.point_step * resp.pointcloud.width;
		resp.pointcloud.data.resize(resp.pointcloud.row_step * resp.pointcloud.height);
		resp.pointcloud.is_dense = false;


		unsigned char* dat = &(resp.pointcloud.data[0]);
		unsigned int n=0;
		for(std::vector<MapPoint*>::iterator it=mpMap->vpPoints.begin(); it!=mpMap->vpPoints.end(); ++it,++n)
		{
			if(n>resp.pointcloud.width-1) break;
			MapPoint p = *(*it);

			Vector<3,float> fvec = p.v3WorldPos;
			uint32_t colorlvl = 0xff<<((3-p.nSourceLevel)*8);
			uint32_t lvl = p.nSourceLevel;
			uint32_t KF = p.pPatchSourceKF->ID;

			memcpy(dat, &(fvec),3*sizeof(float));
			memcpy(dat+3*sizeof(uint32_t),&colorlvl,sizeof(uint32_t));
			memcpy(dat+4*sizeof(uint32_t),&lvl,sizeof(uint32_t));
			memcpy(dat+5*sizeof(uint32_t),&KF,sizeof(uint32_t));
			dat+=resp.pointcloud.point_step;
		}
	}
	return true;
}


bool System::keyframesservice(ptam_com::KeyFrame_srvRequest & req, ptam_com::KeyFrame_srvResponse & resp)
{
	// flags: 	negative number = send newest N KeyFrames
	//			zero = send all available KeyFrames
	//			positive number = send all KeyFrames with ID>N

	ParamsAccess Params;
	double scale = Params.varParams->Scale;

	TooN::SE3<double> pose;
	TooN::Matrix<3, 3, double> rot;
	TooN::Vector<3, double> trans;
	tf::Quaternion q;
	tf::Vector3 t;
	geometry_msgs::PoseWithCovarianceStamped buffpose;
	bool takeKF=false;
	int k=0;
	static unsigned int seq=0;

	if(!(mpMap->vpKeyFrames.size()>0) | !mpMap->bGood)
		return false;

	resp.KFids.reserve(mpMap->vpKeyFrames.size());
	resp.KFs.reserve(mpMap->vpKeyFrames.size());

	for(std::vector<KeyFrame*>::reverse_iterator rit=mpMap->vpKeyFrames.rbegin(); rit!=mpMap->vpKeyFrames.rend();++rit)
	{
		if((req.flags>0) & ((*rit)->ID>=req.flags))
			takeKF=true;
		else if((req.flags<0) & (k<=abs(req.flags)))
			takeKF=true;
		else if(req.flags==0)
			takeKF=true;
		else if((req.flags>0) & ((*rit)->ID<req.flags))
			break;
		else if((req.flags<0) & (k>abs(req.flags)))
			break;

		if(takeKF)
		{
			takeKF=false;
			resp.KFids.push_back((*rit)->ID);
			pose = (*rit)->se3CfromW;
			rot =pose.get_rotation().get_matrix();
			trans = pose.get_translation();
			tf::Transform transform(btMatrix3x3(rot(0, 0), rot(0, 1), rot(0, 2),
																	rot(1, 0), rot(1, 1), rot(1, 2),
																	rot(2, 0), rot(2, 1), rot(2, 2)),
																	btVector3(trans[0] / scale, trans[1]/ scale, trans[2] / scale));
			q = transform.getRotation();
			t = transform.getOrigin();
			buffpose.header.seq=seq;
			buffpose.header.stamp=ros::Time::now();
			buffpose.pose.pose.position.x=t[0];
			buffpose.pose.pose.position.y=t[1];
			buffpose.pose.pose.position.z=t[2];
			buffpose.pose.pose.orientation.w=q.w();
			buffpose.pose.pose.orientation.x=q.x();
			buffpose.pose.pose.orientation.y=q.y();
			buffpose.pose.pose.orientation.z=q.z();
			memset(&(buffpose.pose.covariance[0]),0,sizeof(double)*6*6);
			resp.KFs.push_back(buffpose);

			seq++;
		}
		k++;
	}
	return true;
}


//}

void System::quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R)
{
  // stolen from Eigen3 and adapted to TooN

  TooN::Matrix<3, 3, double> res;

  const double tx = 2 * q.x;
  const double ty = 2 * q.y;
  const double tz = 2 * q.z;
  const double twx = tx * q.w;
  const double twy = ty * q.w;
  const double twz = tz * q.w;
  const double txx = tx * q.x;
  const double txy = ty * q.x;
  const double txz = tz * q.x;
  const double tyy = ty * q.y;
  const double tyz = tz * q.y;
  const double tzz = tz * q.z;

  res(0, 0) = 1 - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1 - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1 - (txx + tyy);

  R = res;

//  R = TooN::SO3<double>::exp(TooN::makeVector<double>(q.x, q.y, q.z) * acos(q.w) * 2.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z));
}



void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if (sCommand == "quit" || sCommand == "exit")
    ros::shutdown();
}








