/*
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: mocap_config.cpp
 *  Desc: Classes representing ROS configuration for mocap_optitrack node. Data
 *  will be published to differed topics based on the configuration provided.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2012, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "mocap_optitrack/mocap_config.h"

const std::string POSE_TOPIC_PARAM_NAME = "pose";
const std::string POSE2D_TOPIC_PARAM_NAME = "pose2d";
const std::string CHILD_FRAME_ID_PARAM_NAME = "child_frame_id";
const std::string PARENT_FRAME_ID_PARAM_NAME = "parent_frame_id";

PublishedRigidBody::PublishedRigidBody(XmlRpc::XmlRpcValue &config_node)
{
  // load configuration for this rigid body from ROS
  publish_pose = validateParam(config_node, POSE_TOPIC_PARAM_NAME);
  publish_pose2d = validateParam(config_node, POSE2D_TOPIC_PARAM_NAME);
  // only publish tf if a frame ID is provided
  publish_tf = (validateParam(config_node, CHILD_FRAME_ID_PARAM_NAME) && 
               validateParam(config_node, PARENT_FRAME_ID_PARAM_NAME));

  if (publish_pose)
  {
    pose_topic = (std::string&) config_node[POSE_TOPIC_PARAM_NAME];
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(pose_topic, 1000);
    rigidbody_state_pub = n.advertise<qrotor_msgs::RigidBodyState>("/qrotor1/rigidbody_state", 1000);

  }

  if (publish_pose2d)
  {
    pose2d_topic = (std::string&) config_node[POSE2D_TOPIC_PARAM_NAME];
    pose2d_pub = n.advertise<geometry_msgs::Pose2D>(pose2d_topic, 1000);
  }

  if (publish_tf)
  {
    child_frame_id = (std::string&) config_node[CHILD_FRAME_ID_PARAM_NAME];
    parent_frame_id = (std::string&) config_node[PARENT_FRAME_ID_PARAM_NAME];
  }

  // custom code
  dt = 0.0;
  IS_INITIALIZED = false;
  alpha = 1.0;
  tau  = 0.02;

}

void PublishedRigidBody::publish(RigidBody &body)
{
  // don't do anything if no new data was provided
  if (!body.has_data())
  {
    return;
  }
  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  // TODO Below was const, see if there a way to keep it like that.
  geometry_msgs::PoseStamped pose = body.get_ros_pose();

  if (publish_pose)
  {
    pose.header.frame_id = parent_frame_id;
    pose_pub.publish(pose);
    rigidbody_state_pub.publish(state);
  }

  if (!publish_pose2d && !publish_tf)
  {
    // nothing to do, bail early
    return;
  }

  tf::Quaternion q(pose.pose.orientation.x,
                   pose.pose.orientation.y,
                   pose.pose.orientation.z,
                   pose.pose.orientation.w);

  // publish 2D pose
  if (publish_pose2d)
  {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;
    pose2d.theta = tf::getYaw(q);
    pose2d_pub.publish(pose2d);
  }

  if (publish_tf)
  {
    // publish transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.pose.position.x,
                                     pose.pose.position.y,
                                     pose.pose.position.z));

    // Handle different coordinate systems (Arena vs. rviz)
    transform.setRotation(q);
    ros::Time timestamp(ros::Time::now());
    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, parent_frame_id, child_frame_id));
  }
}

void PublishedRigidBody::estimate_state(geometry_msgs::PoseStamped &pose)
{

  state.header = pose.header;
  state.position.x = pose.pose.position.x;
  state.position.y = pose.pose.position.y;
  state.position.z = pose.pose.position.z;

  state.attitude.x = pose.pose.orientation.x;
  state.attitude.y = pose.pose.orientation.y;
  state.attitude.z = pose.pose.orientation.z;
  state.attitude.w = pose.pose.orientation.w;


  if (IS_INITIALIZED == false)
  {
    state.velocity.x = 0.0;
    state.velocity.y = 0.0;
    state.velocity.z = 0.0;
    IS_INITIALIZED = true;
  }
  else
  {
    dt = state.header.stamp.toSec() - state_prev.header.stamp.toSec();
    alpha = 1 - exp(-dt/tau);
    
    // finite difference
    dx_dt = (state.position.x - state_prev.position.x)/dt;
    dy_dt = (state.position.y - state_prev.position.y)/dt;
    dz_dt = (state.position.z - state_prev.position.z)/dt;
    
    state.velocity.x = alpha*dx_dt + (1-alpha)*state_prev.velocity.x;
    state.velocity.y = alpha*dy_dt + (1-alpha)*state_prev.velocity.y;
    state.velocity.z = alpha*dz_dt + (1-alpha)*state_prev.velocity.z;

  }



  // storing state for next iteration
  state_prev = state;

}


void PublishedRigidBody::custom_publish(RigidBody &body)
{
  // don't do anything if no new data was provided
  if (!body.has_data())
  {
    return;
  }
  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  // TODO Below was const, see if there a way to keep it like that.
  geometry_msgs::PoseStamped pose = body.get_ros_pose();
  
  PublishedRigidBody::estimate_state(pose);

  if (publish_pose)
  {
    pose.header.frame_id = parent_frame_id;
    pose_pub.publish(pose);
    rigidbody_state_pub.publish(state);

  }

  if (!publish_pose2d && !publish_tf)
  {
    // nothing to do, bail early
    return;
  }

  tf::Quaternion q(pose.pose.orientation.x,
                   pose.pose.orientation.y,
                   pose.pose.orientation.z,
                   pose.pose.orientation.w);

  // publish 2D pose
  if (publish_pose2d)
  {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;
    pose2d.theta = tf::getYaw(q);
    pose2d_pub.publish(pose2d);
  }

  if (publish_tf)
  {
    // publish transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.pose.position.x,
                                     pose.pose.position.y,
                                     pose.pose.position.z));

    // Handle different coordinate systems (Arena vs. rviz)
    transform.setRotation(q);
    ros::Time timestamp(ros::Time::now());
    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, parent_frame_id, child_frame_id));
  }


}

bool PublishedRigidBody::validateParam(XmlRpc::XmlRpcValue &config_node, const std::string &name)
{
  if (!config_node.hasMember(name))
  {
    return false;
  }

  if (config_node[name].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    return false;
  }

  return true;
}

