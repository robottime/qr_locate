#include "iostream"
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"


using namespace std;
tf::TransformBroadcaster* tf_broadcaster;
tf::TransformListener* tf_listener;
string odom_frame_id;
string qr_frame;
string qr_location_topic;
const string kMapFrameId = "map";
tf::StampedTransform odom_in_qr_trans;
tf::StampedTransform qr_in_map_trans;
geometry_msgs::TransformStamped odom_in_map_trans;
bool odom_in_qr_refresh = false;

void callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  //cout<<"msg->detections.length(): "<<msg->detections.size()<<endl;
  if(msg->detections.size()<1)
  {
    return;
  }
  //cout<<"parent_frame: "<<msg->header.frame_id<<endl;
  //cout<<msg->detections[0].pose.pose.pose<<endl;
  geometry_msgs::PoseStamped old_pose;
  geometry_msgs::PoseStamped new_pose;
  old_pose.header = msg->header;
  old_pose.pose = msg->detections[0].pose.pose.pose;
  qr_frame = "tag_"+to_string(msg->detections[0].id[0]);
  try
  {
    tf_listener->transformPose(odom_frame_id,old_pose,new_pose);
    //cout<<"new parent_frame: "<<new_pose.header.frame_id<<endl;
    //cout<<new_pose.pose<<endl;
    tf::Transform it(tf::Quaternion(new_pose.pose.orientation.x,\
                                    new_pose.pose.orientation.y,\
                                    new_pose.pose.orientation.z,\
                                    new_pose.pose.orientation.w),\
                     tf::Vector3(new_pose.pose.position.x,\
                                 new_pose.pose.position.y,\
                                 new_pose.pose.position.z));
    it = it.inverse();
    odom_in_qr_trans.setData(it);
    //cout<<"\nodom_in_qr_trans:"<<endl;
    //cout<<"x:"<<odom_in_qr_trans.getOrigin().x()\
        <<"y:"<<odom_in_qr_trans.getOrigin().y()\
        <<"z:"<<odom_in_qr_trans.getOrigin().z()<<endl;
    double oq_yaw = tf::getYaw(odom_in_qr_trans.getRotation());
    //cout<<oq_yaw<<endl;
    odom_in_qr_refresh = true;
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
  }
}

void broadcastTF(geometry_msgs::TransformStamped& trans)
{
  tf_broadcaster->sendTransform(trans);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"qr_locate");
  ros::NodeHandle nh;
  tf_broadcaster = new tf::TransformBroadcaster();
  tf_listener = new tf::TransformListener();
  nh.param("odom_frame_id",odom_frame_id,string("odom"));
  nh.param("qr_location_topic",qr_location_topic,string("tag_detections"));
  ros::Subscriber sub = nh.subscribe(qr_location_topic,10,&callback);
  odom_in_map_trans.header.frame_id = kMapFrameId;
  odom_in_map_trans.child_frame_id = odom_frame_id;
  odom_in_map_trans.transform.rotation.w = 1;
  ros::Rate rate(100);
  while(!ros::isShuttingDown())
  {
    if(odom_in_qr_refresh)
    {
      try
      {
        tf_listener->lookupTransform(kMapFrameId, qr_frame, ros::Time(0),qr_in_map_trans);
        //cout<<"qr_in_map_trans:"<<endl;
        //cout<<"x:"<<qr_in_map_trans.getOrigin().x()\
            <<"y:"<<qr_in_map_trans.getOrigin().y()\
            <<"z:"<<qr_in_map_trans.getOrigin().z()<<endl;
        double qm_yaw = tf::getYaw(qr_in_map_trans.getRotation());
        //cout<<qm_yaw<<endl;
        tf::Transform qmt(qr_in_map_trans.getRotation(),qr_in_map_trans.getOrigin());
        tf::Transform oqt(odom_in_qr_trans.getRotation(),odom_in_qr_trans.getOrigin());
        tf::Transform omt = qmt*oqt;
        odom_in_map_trans.transform.translation.x = omt.getOrigin().x();
        odom_in_map_trans.transform.translation.y = omt.getOrigin().y();
        odom_in_map_trans.transform.translation.z = omt.getOrigin().z();
        odom_in_map_trans.transform.rotation.x = omt.getRotation().x();
        odom_in_map_trans.transform.rotation.y = omt.getRotation().y();
        odom_in_map_trans.transform.rotation.z = omt.getRotation().z();
        odom_in_map_trans.transform.rotation.w = omt.getRotation().w();
        //cout<<"odom_in_map_trans:"<<endl;
        //cout<<"x:"<<omt.getOrigin().x()\
            <<"y:"<<omt.getOrigin().y()\
            <<"z:"<<omt.getOrigin().z()<<endl;
        double om_yaw = tf::getYaw(omt.getRotation());
        //cout<<om_yaw<<endl;
        odom_in_qr_refresh = false;
      }
      catch (tf::TransformException &e)
      {
      }
    }
    odom_in_map_trans.header.stamp = ros::Time::now();
    broadcastTF(odom_in_map_trans);
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
  return(0);
}