/************************************************************************/
/* ��е�ۿ����࣬Ŀǰ��Robai cyton gramma1500�����ڸû�е�۵�����п�����
   ���е�۵�Ӳ�����*/
/************************************************************************/

#pragma once
#include <foundCore/ecTypes.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <Eigen/Eigen>//���Ӿ�������Ĺ���Eigen������
#include <math.h>//������ѧ���㹤����

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

	///��ȡ��ǰʱ�̵�ĩ��ִ��������ڻ�����ϵ��λ�ˣ�ֱ�����ΪEigen���ݸ�ʽ��7X1������������Ϊλ�ã�3��������һ��ת���ᣨ3������ת����
	VectorXd GetPose();

	///��ȡ��ǰʱ�̵�ĩ��ִ��������ڻ�����ϵ��λ��,����Ϊλ������������������̬����ת������������������̬����ת�Ƕ�ֵ
	void GetPose(EcVector & Position, EcVector & Axis, EcReal & Angle);

	///��ȡ��ǰʱ�̵�ĩ��ִ��������ڻ�����ϵ��λ�ˣ�ֱ�������α任����
	void GetPose(EcReArray &array);

	///��ȡ��ǰʱ�̵ĸ����ؽڽǶȣ�����Ϊ�ؽڽǶ�����

	///�ٶȿ���ģʽ�¶Ի������˶����п��ƣ���Ҫ�����Ӿ��ŷ��˶����ƹ��̣���Ҫ�õ�eigen�����䣬�������㡣����Ϊ�ٶ�ʸ����ǰ������ƽ�ƣ�����������ת
	EcBoolean SetEEVelocity(VectorXd & EEVelo);

	///λ�˿���ģʽ�µĻ����������˶����ƣ���Ҫ������̽���˶�����ȡÿһ����̽������̬�仯����taskfunction�ı仯�����ȡ�ſ˱Ⱦ���ĳ�ֵ,
	///����Ϊ��ǰ��Ҫ���Ƶ����ɶ���š���ǰ�����Ƶ����ɶ��˶����ٶ�,����ֵΪ��Ҫȡֵ��λ�˱仯����
	VectorXd GetEEPoseIncrement(int DoFNum, EcReal DoFVelo);

	///�ٶȿ���ģʽ�µĻ�������̬������ȡ������������ʼ����ֹʱ��pose�����Eigen��ʽ�µ���̬������
	///�����poseΪ7X1���������߸�Ԫ������Ϊλ�ã�3��������һ��ת���ᣨ3������ת���ǣ����Ϊ6X1������������Ϊλ�ñ仯����ǳ˻�
	VectorXd GetEEPoseIncrement(VectorXd Pose_old, VectorXd Pose_new);

	///���йؽڷ�����λ���ұ�֤����֮��ĩ��ִ��������ģʽΪframeģʽ
	EcBoolean Homing();

	//�ٶȿ���ģʽ�»�����ĩ��ִ������ĳһ���ɶȣ���ĳһ�ٶ��£��˶�һ��ʱ��
	//DoFNumΪ���ɶ�1-6�ֱ��ӦX��Y��Z������ת�˶���DoFVeloΪ�ٶȣ�0,1����λΪm/s��msΪ�˶�ʱ�䵥λ����
	void VelocityMoving(int DoFNum, EcReal DoFVelo, EcU32 ms = 1000);

	void changeToFrameEE(EcCoordinateSystemTransformation &desiredPose);

protected:

};