#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

/// Custom range measurement messages
#include "lawnmower/BeaconRangeMsg.h"

#include "trilateration.h"

#define DIM						(2)
#define GLOBAL_FRAME_NAME		"map"
#define ODOM_FRAME_NAME			"odom"
#define RANGE_TOPIC_NAME		"/lawnmower/beacon_ranges"
#define POSITION_TOPIC_NAME		"trilat_position"
#define ODOM_TOPIC_NAME			"trilat_odom"
#define MARKER_TOPIC_NAME		"trilat_marker"
#define LOOP_RATE				(10.0f)
#define THRESH_DIST				(2.0f)
#define THRESH_LIMIT			(50.0f)

//#define DEBUG

using namespace std;

/// Global variables
vector<lawnmower::BeaconRangeMsg> beaconRanges;
bool new_ranges = false;


void rangeCallback(const lawnmower::BeaconRangeMsg& _msg) {
	uint8_t i = 0;
	for (i=0;i<beaconRanges.size();i++) {
		if (beaconRanges[i].id.data == _msg.id.data)
			break;
	}
	if (i >= beaconRanges.size()) {
		beaconRanges.push_back(_msg);
		new_ranges = true;
	}
}


int main(int argc, char** argv) {
	unsigned int nbRanges = 0;
	unsigned int i = 0, id = 0;
	int dim = DIM;
	unsigned int minNbRanges = DIM + 1;
	double loopRate = 0.0f;
	bool trilat_ok = false;
	string global_frame_name;
	string range_topic_name, position_topic_name;
	string odom_topic_name;
	string odom_frame_name;
	string marker_topic_name;
	new_ranges = false;
	beaconRanges.clear();
	vector<float> bx, by, bz;
	vector<float> stds, biases, scales, etas, ids;
	bool first_position = true;
	float thresh_dist0 = THRESH_DIST, thresh_dist = THRESH_DIST;
	float thresh_limit = THRESH_LIMIT;
	float d = 0.0f;
	Eigen::Vector3f dd = Eigen::Vector3f::Zero();
	Eigen::Vector3f p_old = Eigen::Vector3f::Zero();
	unsigned int ct_dist = 0;

	//ofstream log_file;
	//log_file.open("logTrilat.log");

	/// ROS Publishers
	ros::Publisher pub_ptStamped;
	ros::Publisher pub_odom;
	ros::Publisher pub_marker;
	/// ROS Subscribers
	ros::Subscriber sub_beaconRanges;
	/// ROS Messages
	geometry_msgs::PointStamped ptStamped;
	nav_msgs::Odometry odom_msg;
	visualization_msgs::Marker marker_msg;

	/// Init. ROS    
	ros::init(argc, argv, "trilateration_node");
	ros::NodeHandle nh("~");

	if (!nh.getParam ("global_frame_name", global_frame_name))
		global_frame_name = GLOBAL_FRAME_NAME;
	if (!nh.getParam ("odom_frame_name", odom_frame_name))
		odom_frame_name = ODOM_FRAME_NAME;
	if (!nh.getParam ("range_topic_name", range_topic_name))
		range_topic_name = RANGE_TOPIC_NAME;
	if (!nh.getParam ("position_topic_name", position_topic_name))
		position_topic_name = POSITION_TOPIC_NAME;
	if (!nh.getParam ("odom_topic_name", odom_topic_name))
		odom_topic_name = ODOM_TOPIC_NAME;
	if (!nh.getParam ("marker_topic_name", marker_topic_name))
		marker_topic_name = MARKER_TOPIC_NAME;
	if (!nh.getParam ("loop_rate", loopRate))
		loopRate = LOOP_RATE;
	if (!nh.getParam ("dim", dim))
		dim = DIM;
	if (!nh.getParam ("thresh_dist", thresh_dist))
		thresh_dist = THRESH_DIST;
	if (!nh.getParam ("thresh_limit", thresh_limit))
		thresh_limit = THRESH_LIMIT;
	thresh_dist = thresh_dist0;

	nh.getParam ("bx", bx);
	nh.getParam ("by", by);
	nh.getParam ("bz", bz);
	nh.getParam ("stds", stds);
	nh.getParam ("biases", biases);
	nh.getParam ("scales", scales);
	nh.getParam ("etas", etas);
	nh.getParam ("ids", ids);

	if (dim == 2)
		minNbRanges = 3;
	else if (dim == 3)
		minNbRanges = 4;
	else
		cerr << "Bad dimension choose 2D or 3D\n";

	ros::Rate loop_rate(loopRate);
	/// ROS topic subscribers
	sub_beaconRanges = nh.subscribe(range_topic_name, 10, rangeCallback);
	/// ROS topic publishers
	pub_ptStamped = nh.advertise<geometry_msgs::PointStamped>(position_topic_name, loopRate);
	pub_odom = nh.advertise<nav_msgs::Odometry>(odom_topic_name, loopRate);
	pub_marker = nh.advertise<visualization_msgs::Marker>(marker_topic_name, loopRate);

	/// Init. msgs
	ptStamped.header.frame_id = global_frame_name;

	odom_msg.header.frame_id = global_frame_name;
	odom_msg.child_frame_id = odom_frame_name;
	odom_msg.pose.pose.orientation.w = 1.0f;
	odom_msg.pose.pose.orientation.x = 0.0f;
	odom_msg.pose.pose.orientation.y = 0.0f;
	odom_msg.pose.pose.orientation.z = 0.0f;

	marker_msg.header.frame_id = global_frame_name;
	marker_msg.ns = "trilat";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::Marker::SPHERE; // SPHERE
	marker_msg.action = visualization_msgs::Marker::ADD; // MODIFY
	marker_msg.color.a = 1;
	marker_msg.color.r = 0;
	marker_msg.color.g = 1;
	marker_msg.color.b = 0;
	marker_msg.lifetime = ros::Duration(0); // 0 = forever
	marker_msg.frame_locked = true;
	marker_msg.pose.orientation.w = 1.0f;
	marker_msg.pose.orientation.x = 0;
	marker_msg.pose.orientation.y = 0;
	marker_msg.pose.orientation.z = 0;
	marker_msg.scale.x = 0.5;
	marker_msg.scale.y = 0.5;
	marker_msg.scale.z = 0.5;

	Eigen::MatrixXf Xi, C(dim,dim);
	Eigen::VectorXf y, w, x(dim);

	while (ros::ok()) {
		if (new_ranges) {
			new_ranges = false;
			nbRanges = beaconRanges.size();
			if (nbRanges < minNbRanges) {
#ifdef DEBUG
				cerr << "Not enough ranges to perform trilat: dim=";
				cerr << dim << ", nbRanges=" << nbRanges << endl;
#endif
			}
			else {
				Xi = Eigen::MatrixXf(nbRanges, dim);
				y = Eigen::VectorXf(nbRanges);
				w = Eigen::VectorXf(nbRanges);
				/// Prepare data
				for (i=0;i<nbRanges;i++) {
					id = beaconRanges[i].id.data;
					y(i) = beaconRanges[i].range.data;
					if (!stds.empty())
						w(i) = powf(stds[id-1],2.0f);
					else
						w(i) = powf(beaconRanges[i].std.data, 2.0f);
					if (!bx.empty())
						Xi(i,0) = bx[id-1];
					else
						Xi(i,0) = beaconRanges[i].beaconPosition.x;
					if (!by.empty())
						Xi(i,1) = by[id-1];
					else
						Xi(i,1) = beaconRanges[i].beaconPosition.y;
					if (dim == 3) {
						if (!bz.empty())
							Xi(i,2) = bz[id-1];
						else
							Xi(i,2) = beaconRanges[i].beaconPosition.z;
					}
#ifdef DEBUG
					cerr << "B" <<  id << ": r=" << y(i) << ", w= " << w(i);
					cerr << ", p=(" << Xi(i,0) << "," << Xi(i,1);
					if (dim == 3)
						cerr << "," << Xi(i,2);
					cerr << ")\n";
#endif
				}
				trilat_ok = linearTrilateration(y, w, x, Xi, C);
				x = nonlinearTrilateration(x, y, Xi, w);
				if (trilat_ok && (fabs(x(0))<thresh_limit) && 
								(fabs(x(1))<thresh_limit)) {
#ifdef DEBUG
					cerr << "x=" << x.transpose() << endl;
					cerr << "C=" << C << endl;
#endif
					if (first_position)
						first_position = false;
						p_old(0) = x(0);
						p_old(1) = x(1);
						if (dim == 3)
							p_old(2) = x(2);
					else {
						dd(0) = x(0) - p_old(0);
						dd(1) = x(1) - p_old(1);
						if (dim ==3) {
							dd(2) = x(2) - p_old(2);
							d = sqrtf(dd(0)*dd(0) + dd(1)*dd(1) + dd(2)*dd(2));
						}
						else
							d = sqrtf(dd(0)*dd(0) + dd(1)*dd(1));
						if (d < thresh_dist) {
							//log_file << x(0) << "," << x(1);
							ptStamped.header.seq++;
							ptStamped.header.stamp = ros::Time::now();
							odom_msg.header.seq++;
							odom_msg.header.stamp = ros::Time::now();
							marker_msg.header.seq++;
							marker_msg.header.stamp = ros::Time::now();
							marker_msg.id++;
							ptStamped.point.x = x(0);
							ptStamped.point.y = x(1);
							odom_msg.pose.pose.position.x = x(0);
							odom_msg.pose.pose.position.y = x(1);
							marker_msg.pose.position.x = x(0);
							marker_msg.pose.position.y = x(1);

							if (dim == 3) {
								//log_file << "," << x(2);
								ptStamped.point.z = x(2);
								odom_msg.pose.pose.position.z = x(2);
								marker_msg.pose.position.z = x(2);
							}
							else {
								ptStamped.point.z = 0;
								odom_msg.pose.pose.position.z = 0;
								marker_msg.pose.position.z = 0;
							}
							//log_file << endl;
							pub_ptStamped.publish(ptStamped);
							pub_odom.publish(odom_msg);
							pub_marker.publish(marker_msg);
							ct_dist = 0;
							thresh_dist = thresh_dist0;
						}
						else {
							ct_dist++;
							thresh_dist += ((1.f*ct_dist)/loopRate);
						}
					}
				}
				else {
#ifdef DEBUG
					cerr << "Trilat. error\n";
#endif
				}
			}
			beaconRanges.clear();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	//log_file.close();

	return 0;
}
