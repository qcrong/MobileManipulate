/************************************************************************/
/* 机械臂控制类，目前针Robai cyton gramma1500，基于该机械臂的类进行开发，
   与机械臂的硬件相关*/
/************************************************************************/

#pragma once
#include <foundCore/ecTypes.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <Eigen/Eigen>//增加矩阵运算的工具Eigen工具箱
#include <math.h>//增加数学运算工具箱

#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <iostream>

#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

namespace bpo = boost::program_options;

using namespace Eigen;
using namespace std;

/// This class uses the remote commands API to communicate with the 
//  ActinViewer/CytonViewer or ActinRT with the remoteCommandServerPlugin loaded.
class EcCytonCommands
{
public:

	/// constructor
	EcCytonCommands
		(
		);

	/// destructor
	virtual ~EcCytonCommands
		(
		);

	/// copy constructor
	EcCytonCommands
		(
		const EcCytonCommands& orig
		);

	/// overloading = operator
	const EcCytonCommands& operator=
		(
		const EcCytonCommands& orig
		)const;

	/// overloading == operator
	EcBoolean operator==
		(
		const EcCytonCommands& orig
		)const;

	/// initialize the network
	/// @param[in] ipAddress          (EcString&) address of the network to be connected to.
	/// @return                       (EcBoolean) which returns the status of command
	virtual EcBoolean openNetwork
		(
		const EcString& ipAddress
		)const;

	/// shut down the network
	/// @return    flag (EcBoolean) which returns the status of command
	virtual EcBoolean closeNetwork
		(
		)const;

	/// test joint values communication over the network
	/// @param[in] jointPosition (EcRealVector&) desired joint state
	/// @param[in] angletolerance (EcReal&) desired joint state precision
	/// @return    flag  (EcBoolean) which returns the status of command
	virtual EcBoolean MoveJointsExample
		(
		const EcRealVector jointPosition,
		const EcReal angletolerance
		)const;

	/// Move the gripper on the server side
	/// @param[in] gripperPos  (EcReal) desired gripper position
	/// @return    flag              (EcBoolean) which returns the status of command
	virtual EcBoolean moveGripperExample
		(
		const EcReal gripperPos
		)const;

	/// Test executing manipulation actions
	/// @param[in] filename (EcString&) manipulation action manager filename
	/// @param[in] actionName (EcString&) manipulation action name to run
	/// @return    flag     (EcBoolean) which returns the status of command
	virtual EcBoolean manipulationActionTest
		(
		const EcString& filename,
		const EcString& actionName
		)const;

	/// Test executing manipulation action series
	/// @param[in] filename (EcString&) manipulation action manager filename
	/// @return    flag     (EcBoolean) which returns the status of command
	virtual EcBoolean manipulationActionSeriesTest
		(
		const EcString& filename
		)const;

	/// Test executing manipulation director script
	/// @param[in] filename (EcString&) manipulation action manager filename
	/// @return    flag     (EcBoolean) which returns the status of command
	virtual EcBoolean manipulationDirectorTest
		(
		const EcString& filename
		)const;

	/// move the robot using point EE set (only constrains position (x,y,z))
	/// @param[in] pose (EcCoordinateSystemTransformation&) desired pose
	/// @return         (EcBoolean) flag which returns the status of command
	virtual EcBoolean pointMovementExample
		(
		const EcCoordinateSystemTransformation& pose
		)const;

	/// move the robot using frame EE set (constrains x,y,z, and roll, pitch, yaw)
	/// @param[in] pose (EcCoordinateSystemTransformation&) desired pose
	/// @return         (EcBoolean) flag which returns the status of command
	virtual EcBoolean frameMovementExample
		(
		const EcCoordinateSystemTransformation& pose
		)const;


	///sample path planning to get to the desired pose
	// @param[in] pose (EcCoordinateSystemTransformation&) desired Pose
	/// @return         (EcBoolean) flag which returns the status of command
	virtual EcBoolean pathPlanningExample
		(
		const EcCoordinateSystemTransformation& pose
		)const;

	/// move the end effector with the desired end effector velocity for one second
	/// @param[in] endVelo (EcRealVector&) desired velocity direction and magnitude [x,y,z]
	virtual EcBoolean endEffectorVelocityTest
		(
		const EcRealVector& endVelo
		)const;

	/// enable hardware test
	/// @param[in] flag     (EcBoolean) Whether to enable/disable hardware
	/// @return    flag     (EcBoolean)which returns the status of command
	virtual EcBoolean hardwareEnableTest
		(
		const EcBoolean flag
		)const;

	/// move the robot to home position(zero joint angles for all the joints)
	virtual EcBoolean resetToHome
		(
		)const;

	///获取当前时刻的末端执行器相对于基坐标系的位姿，直接输出为Eigen数据格式下7X1的向量，依次为位置（3个）、归一化转动轴（3个）、转动角
	VectorXd GetPose();

	///获取当前时刻的末端执行器相对于基坐标系的位姿,输入为位置向量，用于描述姿态的旋转轴向量，用于描述姿态的旋转角度值
	void GetPose(EcVector & Position, EcVector & Axis, EcReal & Angle);

	///获取当前时刻的末端执行器相对于基坐标系的位姿，直接输出齐次变换矩阵
	void GetPose(EcReArray &array);

	///获取当前时刻的各个关节角度，输入为关节角度向量

	///速度控制模式下对机器人运动进行控制，主要用于视觉伺服运动控制过程，需要用到eigen工具箱，方便运算。输入为速度矢量，前三个是平移，后三个是旋转
	EcBoolean SetEEVelocity(VectorXd & EEVelo);

	///位姿控制模式下的机器人增量运动控制，主要用于试探步运动，获取每一个试探步的姿态变化，与taskfunction的变化结合求取雅克比矩阵的初值,
	///输入为当前需要控制的自由度序号、当前所控制的自由度运动的速度,返回值为需要取值的位姿变化向量
	VectorXd GetEEPoseIncrement(int DoFNum, EcReal DoFVelo);

	///速度控制模式下的机器人姿态增量获取，输入周期起始与终止时的pose，输出Eigen格式下的姿态增量，
	///输入的pose为7X1的向量，七个元素依次为位置（3个）、归一化转动轴（3个）、转动角；输出为6X1的向量，依次为位置变化、轴角乘积
	VectorXd GetEEPoseIncrement(VectorXd Pose_old, VectorXd Pose_new);

	///所有关节返回零位，且保证返回之后，末端执行器控制模式为frame模式
	EcBoolean Homing();

	//速度控制模式下机器人末端执行器沿某一自由度，在某一速度下，运动一定时间
	//DoFNum为自由度1-6分别对应X、Y、Z及其旋转运动；DoFVelo为速度（0,1】单位为m/s，ms为运动时间单位毫秒
	void VelocityMoving(int DoFNum, EcReal DoFVelo, EcU32 ms = 1000);

	void changeToFrameEE(EcCoordinateSystemTransformation &desiredPose);

protected:

};