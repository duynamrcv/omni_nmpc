#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "acado.h"

// /* Global variables used by the solver. */
// ACADOvariables acadoVariables;
// ACADOworkspace acadoWorkspace;

nav_msgs::Odometry odom;
void stateCallback(const nav_msgs::Odometry& msg) { odom = msg; }

nav_msgs::Path path;
void pathCallback(const nav_msgs::Path& msg) { path = msg; }

bool is_target(nav_msgs::Odometry cur, double goal_x, double goal_y)
{
	if(abs(cur.pose.pose.position.x - goal_x) < 0.05 && abs(cur.pose.pose.position.y - goal_y) < 0.05)
	{
		return true;
	}
	else return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_node");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe("/odom", 10, stateCallback);
    ros::Subscriber path_sub = nh.subscribe("/path", 1, pathCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Publisher predict_pub = nh.advertise<nav_msgs::Path>("/predict_path", 10);
	ros::Publisher odom_path_pub = nh.advertise<nav_msgs::Path>("/odom_path_pub", 10);

	// /* ROS PARAM */
	// ros::param::get("~weight_x", weight_x);
	// ros::param::get("~weight_y", weight_y);
	// ros::param::get("~weight_q", weight_q);
	// ros::param::get("~weight_vx", weight_vx);
	// ros::param::get("~weight_vy", weight_vy);
	// ros::param::get("~weight_w", weight_w);

	// cout << weight_x << " " << weight_y << " " << weight_q << endl;

	// Odom path 
	nav_msgs::Path odom_path;
	odom_path.header.frame_id = "/odom";
	odom_path.header.stamp = ros::Time::now(); 	

    vector<vector<double>> control_output;
    control_output = init_acado();

    // cout << control_output[0][0] << endl;
	double start_time = ros::Time::now().toSec();
    ros::Rate r(10);

	double goal_x, goal_y;
	int count = 0;
    while(ros::ok())
    {
		if (path.poses.size() == 0)
		{
			ros::spinOnce();
			r.sleep();
			continue;
		}

		goal_x = path.poses[path.poses.size()-1].pose.position.x;
		goal_y = path.poses[path.poses.size()-1].pose.position.y;

		// State
		double px = odom.pose.pose.position.x; // state[0];
		double py = odom.pose.pose.position.y; // state[1];

		double q0 = odom.pose.pose.orientation.x;
		double q1 = odom.pose.pose.orientation.y;
		double q2 = odom.pose.pose.orientation.z;
		double q3 = odom.pose.pose.orientation.w;
		double t0 = -2.0 * (q1*q1 + q2*q2) + 1.0;
		double t1 = +2.0 * (q2*q3 + q0*q1);

		double pq = atan2(t1, t0); // state[2]; 

		vector<double> cur_state = {px, py, pq};

		// Update odom path
		geometry_msgs::PoseStamped cur_pose;
		cur_pose.header = odom_path.header;
		cur_pose.pose.position.x = px;
		cur_pose.pose.position.y = py;
		cur_pose.pose.orientation.w = 1.0;

		odom_path.poses.push_back(cur_pose);
		
		odom_path_pub.publish(odom_path);

		// Reference state
		vector<double> ptsx, ptsy, ptsq;

		for (int i = 0; i < ACADO_N; i++)
		{			
			double pred_x, pred_y;
			if (count+i >= path.poses.size())
			{
				pred_x = path.poses[path.poses.size()-1].pose.position.x;
				pred_y = path.poses[path.poses.size()-1].pose.position.y;
				ptsx.push_back(pred_x);
				ptsy.push_back(pred_y);
				ptsq.push_back(0);
			}
			else
			{
				pred_x = path.poses[count+i].pose.position.x;
				pred_y = path.poses[count+i].pose.position.y;
				ptsx.push_back(pred_x);
				ptsy.push_back(pred_y);
				if(i == 0) ptsq.push_back(pq);
				else
				{
					double delta_x = path.poses[count+i].pose.position.x - path.poses[count+i-1].pose.position.x;
					double delta_y = path.poses[count+i].pose.position.y - path.poses[count+i-1].pose.position.y;
					double theta = atan2(delta_y, delta_x);
					ptsq.push_back(theta);
				}
			}
		}

		double reference_vx = 0.0;
		double reference_vy = 0.0;
		double reference_w = 0.0;

        // ACADO
        vector<double> predicted_states = motion_prediction(cur_state, control_output);
        vector<double> ref_states = calculate_ref_states(ptsx, ptsy, ptsq, reference_vx, reference_vy, reference_w);
        control_output = run_mpc_acado(predicted_states, ref_states, control_output);

		// Predict path
		nav_msgs::Path predict_path;
		predict_path.header.frame_id = "/odom";
		predict_path.header.stamp = ros::Time::now();
		for (int i = 0; i < ACADO_N; i++)
		{
			geometry_msgs::PoseStamped pred_pose;
			pred_pose.header = predict_path.header;
			pred_pose.pose.position.x = acadoVariables.x[NX * i + 0];
			pred_pose.pose.position.y = acadoVariables.x[NX * i + 1];
			pred_pose.pose.orientation.w = 1.0;
			predict_path.poses.push_back(pred_pose);
		}
		predict_pub.publish(predict_path);
		// cout << "Oke" << endl;

		geometry_msgs::Twist vel;

		// Check target
		bool goal = is_target(odom, goal_x, goal_y) && count >= path.poses.size();
		if(goal)
		{
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.angular.z = 0;
			// cout << "Done!" << endl;
		}
		else
		{
			vel.linear.x = control_output[0][0];
			vel.linear.y = control_output[1][0];
			vel.angular.z = control_output[2][0];
		}
		vel_pub.publish(vel);

        ros::spinOnce();
        r.sleep();
		count++;
    }
    return 0;
}