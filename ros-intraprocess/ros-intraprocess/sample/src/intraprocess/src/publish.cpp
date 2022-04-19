#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <stdio.h>

namespace nodelet_intraprocess_publish
{

	class Publish : public nodelet::Nodelet
	{
	public:
	  Publish()
	  : m_count(0)
	  {}

	private:
	  virtual void onInit()
	  {
		ros::NodeHandle& private_nh = getPrivateNodeHandle();
		m_pub = private_nh.advertise<std_msgs::String>("intraprocess_topic", 10);
		m_timer = private_nh.createTimer(ros::Duration(1), boost::bind(&Publish::timerCallback, this, _1));
	  }

	  void timerCallback(const ros::TimerEvent &event)
	  {
		std_msgs::StringPtr output(new std_msgs::String()); 
		output->data = "hello world " + std::to_string(m_count++);
		ROS_INFO("(Publisher)data:%s, address:%p, pid:%d", output->data.c_str(), &(output->data), getpid());
		m_pub.publish(output);
	  }

	  ros::Publisher m_pub;
	  ros::Timer m_timer;
	  int m_count;
	};

	PLUGINLIB_EXPORT_CLASS(nodelet_intraprocess_publish::Publish, nodelet::Nodelet)
}

