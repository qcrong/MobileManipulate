/************************************************************************/
/* ��е�ۿ����࣬Ŀǰ��Robai gramma1500�����ڸû�е�۵�����п�����
���е�۵�Ӳ�����*/
/************************************************************************/

#include "MmArmBase.h"
#include <control/ecEndEffectorSet.h>
#include <controlCore/ecFrameEndEffector.h>
#include <control/ecManipEndEffectorPlace.h>
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <manipulation/ecManipulationActionManager.h>
#include <manipulationDirector/ecManipulationScript.h>
#include <manipulationDirector/ecManipulationDirector.h>
#include <remoteCommand/ecRemoteCommand.h>
#include <xmlReaderWriter/ecXmlObjectReaderWriter.h>
#include <boost/bind.hpp>

#define POINT_EE_SET 0
#define FRAME_EE_SET 1
#define JOINT_CONTROL_EE_SET 0xFFFFFFFF

//------------------------------------------------------------------------------
// Callback function.  Sets a local variable with completion status.
static EcBoolean g_ManipulationComplete = EcFalse;
void manipCallback(EcBoolean status, void* data)
{
	std::cout << "Received sequence completed status of " << (status ? "SUCCESS" : "FAILURE") << std::endl;
	g_ManipulationComplete = EcTrue;
}

class Wait
{
public:
	EcBoolean waitForCompletion
		(
		)
	{
		std::cout << "Waiting for manipulation to complete" << std::endl;
		EcMutexScopedLock lock(m_Mutex);
		m_CompletionReceived = EcFalse;
		while (!m_CompletionReceived)
		{
			m_Condition.wait(lock);
		}
		return m_Success;
	}

	void actionExecCompletionCallback
		(
		EcBoolean success,
		void* // data
		)
	{
		{
			EcMutexScopedLock lock(m_Mutex);
			m_CompletionReceived = EcTrue;
			m_Success = success;
		}
		m_Condition.notify_all();
	}

private:
	EcMutex     m_Mutex;
	EcCondition m_Condition;
	EcBoolean   m_CompletionReceived;
	EcBoolean   m_Success;
};

using namespace Ec;
//----------------------------------constructor--------------------------------
EcCytonCommands::EcCytonCommands
(
)
{
}

//----------------------------------destructor---------------------------------
EcCytonCommands::~EcCytonCommands
(
)
{
}

//----------------------------------overloading = -----------------------------
const EcCytonCommands& EcCytonCommands:: operator=
(
const EcCytonCommands& orig
)const
{
	return *this;
}

//----------------------------------overloading == ----------------------------
EcBoolean EcCytonCommands:: operator==
(
const EcCytonCommands& orig
)const
{
	return EcTrue;
}

//----------------------------------open network-------------------------------
EcBoolean EcCytonCommands::openNetwork
(
const EcString& m_IpAddress
)const
{
	EcBoolean retVal = EcTrue;
	if (!init(m_IpAddress))
	{
		std::cerr << "Failed to init\n";
		return EcFalse;
	}
	retVal &= setManipulationCompletedCallback(manipCallback);
	return EcTrue;
}

//----------------------------------close network------------------------------
EcBoolean EcCytonCommands::closeNetwork
(
)const
{
	shutdown();
	return EcTrue;
}

//----------------------------------joint commands test------------------------

EcBoolean EcCytonCommands::MoveJointsExample
(
const EcRealVector jointPosition,
const EcReal angletolerance
)const
{
	EcBoolean retVal = EcTrue;
	setEndEffectorSet(JOINT_CONTROL_EE_SET);
	EcSLEEPMS(500);

	//vector of EcReals that holds the set of joint angles
	EcRealVector currentJoints;
	retVal &= getJointValues(currentJoints);

	size_t size = currentJoints.size();
	if (size < jointPosition.size())
	{
		size = currentJoints.size();
	}
	else if (size >= jointPosition.size())
	{
		size = jointPosition.size();
	}

	EcPrint(Debug) << "Current Joint Angles: ( ";
	for (size_t ii = 0; ii < size; ++ii)
	{
		EcPrint(Debug) << currentJoints[ii] << ",";
		currentJoints[ii] = jointPosition[ii];
	}
	EcPrint(Debug) << " )" << std::endl;

	std::cout << "Desired joint Angles: ( ";
	for (size_t ii = 0; ii < size; ++ii)
	{
		std::cout << currentJoints[ii] << ",";
	}
	std::cout << " )" << std::endl;

	retVal &= setJointValues(currentJoints);

	//Check if achieved
	EcBooleanVector jointAchieved;
	jointAchieved.resize(size);
	EcBoolean positionAchieved = EcFalse;

	// if it hasnt been achieved after 5 sec, return false
	EcU32 timeout = 5000;
	EcU32 interval = 10;
	EcU32 count = 0;

	while (!positionAchieved && !(count >= timeout / interval))
	{
		EcSLEEPMS(interval);
		count++;

		EcPrint(Debug) << "Moving ";
		getJointValues(currentJoints);
		EcPrint(Debug) << "Current Joints: ";
		for (size_t ii = 0; ii < size; ++ii)
		{

			EcPrint(Debug) << " , " << currentJoints[ii];

			if (std::abs(jointPosition[ii] - currentJoints[ii]) < angletolerance)
			{
				jointAchieved[ii] = EcTrue;
			}
		}
		EcPrint(Debug) << std::endl;
		for (size_t ii = 0; ii < size; ++ii)
		{
			if (!jointAchieved[ii])
			{
				positionAchieved = EcFalse;
				break;
			}
			else
			{
				positionAchieved = EcTrue;
			}
		}
	}
	EcPrint(Debug) << " Final Joint Angles: (";
	for (size_t ii = 0; ii < size; ++ii)
	{
		EcPrint(Debug) << currentJoints[ii] << ",";
	}
	EcPrint(Debug) << " ) " << std::endl;


	std::cout << (positionAchieved ? "Achieved Joint State" : "Failed to Achieve Joint State") << std::endl;

	return positionAchieved;
}

//-----------------------------Point Movement Example-------------------------
EcBoolean EcCytonCommands::pointMovementExample
(
const EcCoordinateSystemTransformation& pose
)const
{

	std::cout << "Desired pose:  x: " << pose.translation().x() << " y: " << pose.translation().y() << " z: " << pose.translation().z() << std::endl;

	setEndEffectorSet(POINT_EE_SET); // point end effector set index
	EcEndEffectorPlacement desiredPlacement(pose);
	EcManipulatorEndEffectorPlacement actualEEPlacement;
	EcCoordinateSystemTransformation offset, zero, actualCoord;
	zero.setTranslation(EcVector(0, 0, 0));

	//set the desired position
	setDesiredPlacement(desiredPlacement, 0, 0);

	// if it hasnt been achieved after 5 sec, return false
	EcU32 timeout = 5000;
	EcU32 interval = 10;
	EcU32 count = 0;
	EcBoolean achieved = EcFalse;
	while (!achieved && !(count >= timeout / interval))
	{
		EcSLEEPMS(interval);
		count++;

		EcPrint(Debug) << "Moving " << std::endl;
		getActualPlacement(actualEEPlacement);
		if (actualEEPlacement.offsetTransformations().size() < 1)
		{
			return EcFalse;
		}
		actualCoord = actualEEPlacement.offsetTransformations()[0].coordSysXForm();

		//get the transformation between the actual and desired 
		offset = (actualCoord.inverse()) * pose;
		EcPrint(Debug) << "distance between actual and desired: " << offset.translation().mag() << std::endl;

		if (offset.approxEq(zero, .00001))
		{
			achieved = EcTrue;
		}

	}
	std::cout << (achieved ? "Achieved Pose" : "Failed to Achieve Pose") << std::endl;
	return achieved;
}

//-----------------------------Frame Movement Example-------------------------
EcBoolean EcCytonCommands::frameMovementExample
(
const EcCoordinateSystemTransformation& pose
)const
{

	std::cout << "Desired pose:  x: " << pose.translation().x() << " y: " << pose.translation().y() << " z: " << pose.translation().z() << std::endl;

	setEndEffectorSet(FRAME_EE_SET); // frame end effector set index
	EcEndEffectorPlacement desiredPlacement(pose);
	EcManipulatorEndEffectorPlacement actualEEPlacement;
	EcCoordinateSystemTransformation offset, zero, actualCoord;
	zero.setTranslation(EcVector(0, 0, 0));

	getActualPlacement(actualEEPlacement);
	EcEndEffectorPlacementVector state = actualEEPlacement.offsetTransformations();
	state[0] = desiredPlacement;
	//set the desired position
	setDesiredPlacement(desiredPlacement, 0, 0);

	// if it hasnt been achieved after 5 sec, return false
	EcU32 timeout = 5000;
	EcU32 interval = 10;
	EcU32 count = 0;
	EcBoolean achieved = EcFalse;
	while (!achieved && !(count >= timeout / interval))
	{
		EcSLEEPMS(interval);
		count++;

		EcPrint(Debug) << "Moving " << std::endl;
		getActualPlacement(actualEEPlacement);
		actualCoord = actualEEPlacement.offsetTransformations()[0].coordSysXForm();

		//get the transformation between the actual and desired 
		offset = (actualCoord.inverse()) * pose;
		EcPrint(Debug) << "distance between actual and desired: " << offset.translation().mag() << std::endl;

		if (offset.approxEq(zero, .00001))
		{
			EcPrint(Debug) << "Achieved Pose" << std::endl;
			achieved = EcTrue;
		}
	}
	std::cout << (achieved ? "Achieved Pose" : "Failed to Achieve Pose") << std::endl;
	return achieved;
}

//-----------------------------move gripper test-------------------------
EcBoolean EcCytonCommands::moveGripperExample
(
const EcReal gripperPos
)const
{
	EcManipulatorEndEffectorPlacement actualEEPlacement, desiredEEPlacement;
	//switch to frame ee set, so the link doesnt move when we try and grip
	setEndEffectorSet(FRAME_EE_SET);
	EcSLEEPMS(100);
	//get the current placement of the end effectors
	getActualPlacement(actualEEPlacement);

	//0 is the Wrist roll link (point or frame end effector), 
	//1 is the first gripper finger link (linear constraint end effector)
	EcEndEffectorPlacementVector state = actualEEPlacement.offsetTransformations();

	if (state.size() < 2)
	{
		// The server isn't connected to a robot.
		return EcFalse;
	}

	//set the translation of the driving gripper finger
	EcCoordinateSystemTransformation gripperfinger1trans = state[1].coordSysXForm();
	gripperfinger1trans.setTranslation(EcVector(0, 0, gripperPos));
	EcEndEffectorPlacement finger1placement = state[1];
	finger1placement.setCoordSysXForm(gripperfinger1trans);
	state[1] = finger1placement;

	desiredEEPlacement.setOffsetTransformations(state);

	//set the desired placement
	setDesiredPlacement(desiredEEPlacement, 0);

	// if it hasnt been achieved after 2 sec, return false
	EcU32 timeout = 2000;
	EcU32 interval = 10;
	EcU32 count = 0;
	EcBoolean achieved = EcFalse;
	while (!achieved && !(count >= timeout / interval))
	{
		EcSLEEPMS(interval);
		count++;

		EcPrint(Debug) << "Moving " << std::endl;

		getActualPlacement(actualEEPlacement);
		EcEndEffectorPlacementVector currentState = actualEEPlacement.offsetTransformations();
		EcCoordinateSystemTransformation gripperfinger1trans = currentState[1].coordSysXForm();
		EcReal difference = std::abs(gripperPos - gripperfinger1trans.translation().z());
		EcPrint(Debug) << "distance between actual and desired: " << difference << std::endl;

		if (difference < .000001)
		{
			achieved = EcTrue;
		}
	}
	std::cout << (achieved ? "Achieved Gripper Position" : "Failed to Achieve Gripper Position") << std::endl;
	return achieved;
}

//-----------------------------manipulation action test-------------------------
EcBoolean EcCytonCommands::manipulationActionTest
(
const EcString& filename,
const EcString& actionName
)const
{
	EcManipulationActionManager actionManager;
	EcBoolean retVal = EcXmlObjectReaderWriter::readFromFile(actionManager, filename);

	if (!retVal)
	{
		return retVal;
	}

	retVal = setManipulationActionManager(actionManager);

	EcManipulationScript script;
	setManipulationScript(script);

	if (!retVal)
	{
		return retVal;
	}

	retVal = setManipulationAction(actionName);
	if (!retVal)
	{
		return retVal;
	}

	retVal = startManipulation();
	if (!retVal)
	{
		return retVal;
	}

	Wait wait;
	setManipulationCompletedCallback(boost::bind(&Wait::actionExecCompletionCallback, &wait, _1, _2));

	// wait
	retVal = wait.waitForCompletion();

	return retVal;
}

EcBoolean EcCytonCommands::manipulationActionSeriesTest
(
const EcString& filename
)const
{
	EcManipulationActionManager actionManager;
	EcBoolean retVal = EcXmlObjectReaderWriter::readFromFile(actionManager, filename);

	if (!retVal)
	{
		return retVal;
	}

	retVal = setManipulationActionManager(actionManager);

	EcManipulationScript script;
	setManipulationScript(script);

	if (!retVal)
	{
		return retVal;
	}

	EcXmlStringVector actionOrderList;

	actionOrderList = actionManager.actionOrder();
	EcU32 numActions = (EcU32)actionOrderList.size();
	for (EcU32 ii = 0; ii < numActions; ++ii)
	{

		retVal = setManipulationAction(actionOrderList[ii].value());

		if (!retVal)
		{
			return retVal;
		}

		retVal = startManipulation();
		if (!retVal)
		{
			return retVal;
		}

		Wait wait;
		setManipulationCompletedCallback(boost::bind(&Wait::actionExecCompletionCallback, &wait, _1, _2));

		// wait
		retVal = wait.waitForCompletion();
	}
	return retVal;
}

EcBoolean EcCytonCommands::manipulationDirectorTest
(
const EcString& filename
)const
{

	EcManipulationDirector director;
	EcBoolean retVal = EcXmlObjectReaderWriter::readFromFile(director, filename);

	if (!retVal)
	{
		return retVal;
	}
	std::cout << "loaded director:\n";
	retVal = setManipulationDirector(director);

	if (!retVal)
	{
		return retVal;
	}
	std::cout << "set director:\n";

	retVal = startManipulation();
	if (!retVal)
	{
		return retVal;
	}
	std::cout << "started director:\n";

	Wait wait;
	setManipulationCompletedCallback(boost::bind(&Wait::actionExecCompletionCallback, &wait, _1, _2));

	// wait
	retVal = wait.waitForCompletion();

	return retVal;
}

EcBoolean EcCytonCommands::pathPlanningExample
(
const EcCoordinateSystemTransformation& pose
)const
{
	EcBoolean retVal = EcTrue;
	// run this the first time
	std::cout << "Running path planning" << std::endl;

	EcManipulatorEndEffectorPlacement actualEEPlacement, desiredEEPlacement;
	//switch to frame ee set, so the link doesnt move when we try and grip
	setEndEffectorSet(FRAME_EE_SET);
	EcSLEEPMS(100);
	//get the current placement of the end effectors
	getActualPlacement(actualEEPlacement);

	desiredEEPlacement = actualEEPlacement;

	desiredEEPlacement.offsetTransformations()[0].setCoordSysXForm(pose);
	retVal = setPathPlanningDesiredPlacement(desiredEEPlacement);

	if (!retVal)
	{
		return EcFalse;
	}

	Wait wait;
	setManipulationCompletedCallback(boost::bind(&Wait::actionExecCompletionCallback, &wait, _1, _2));

	// wait
	retVal = wait.waitForCompletion();

	return retVal;
}

EcBoolean EcCytonCommands::endEffectorVelocityTest
(
const EcRealVector &endVelo
) const
{
	// change to velocity control
	EcBoolean retVal = setControlMode(1);
	if (retVal)
	{
		// wait 100 ms to take effect
		EcSLEEPMS(100);
		retVal = setDesiredVelocity(endVelo);

		// wait 1 second and then send zero velocity so it stops
		EcSLEEPMS(1000);

		EcRealVector zeroVelo(endVelo.size());
		retVal = setDesiredVelocity(zeroVelo);

		// change back to position control
		retVal = setControlMode(0);
	}

	return retVal;
}



EcBoolean EcCytonCommands::hardwareEnableTest
(
const EcBoolean flag
)const
{
	return setHardwareEnable(flag);
}

EcBoolean EcCytonCommands::resetToHome
(
)const
{
	EcRealVector joints;
	EcBoolean retVal = getJointValues(joints);

	size_t size = joints.size();

	// increment all joints except the last
	for (size_t ii = 0; ii < size; ++ii)
	{
		joints[ii] = 0.0;
	}

	retVal &= setJointValues(joints);
	EcSLEEPMS(2000);

	return retVal;
}

///��ȡ��ǰʱ�̵�ĩ��ִ��������ڻ�����ϵ��λ��,����Ϊλ������������������̬����ת������������������̬����ת�Ƕ�ֵ
void EcCytonCommands::GetPose(EcVector & Position, EcVector & Axis, EcReal & Angle)
{
	setEndEffectorSet(FRAME_EE_SET);//��������ϵSET����֤�ܹ���ȡλ��
	EcManipulatorEndEffectorPlacement actualEEPlacement;
	getActualPlacement(actualEEPlacement);
	EcCoordinateSystemTransformation actualCoord;
	actualCoord = actualEEPlacement.offsetTransformations()[0].coordSysXForm();
	Position = actualCoord.translation();//λ��
	actualCoord.orientation().getAngleAxis(Angle, Axis);//�������
}

VectorXd EcCytonCommands::GetPose()
{
	EcVector Position, Axis, oriention;
	EcReal  Angle, phi, theta, psi;
	setEndEffectorSet(FRAME_EE_SET);//��������ϵSET����֤�ܹ���ȡλ��
	EcSLEEPMS(100);
	EcManipulatorEndEffectorPlacement actualEEPlacement;
	getActualPlacement(actualEEPlacement);
	EcCoordinateSystemTransformation actualCoord;
	actualCoord = actualEEPlacement.offsetTransformations()[0].coordSysXForm();


	Position = actualCoord.translation();//λ��
	actualCoord.orientation().get321Euler(psi, theta, phi);
	actualCoord.orientation().getAngleAxis(Angle, Axis);//�������

	VectorXd Pose(6);
	//	Pose << Position.x(), Position.y(), Position.z(), Axis.x(), Axis.y(), Axis.z(), Angle;
	Pose << Position.x(), Position.y(), Position.z(), phi, theta, psi;
	return Pose;
}

void EcCytonCommands::GetPose(EcReArray & array)
{
	setEndEffectorSet(FRAME_EE_SET);//��������ϵSET����֤�ܹ���ȡλ��
	EcSLEEPMS(100);
	EcManipulatorEndEffectorPlacement actualEEPlacement;
	getActualPlacement(actualEEPlacement);
	EcCoordinateSystemTransformation actualCoord;
	actualCoord = actualEEPlacement.offsetTransformations()[0].coordSysXForm();
	array = actualCoord.make4x4();
}

///�ٶȿ���ģʽ�¶Ի������˶����п��ƣ���Ҫ�����Ӿ��ŷ��˶����ƹ��̣���Ҫ�õ�eigen�����䣬�������㡣����Ϊ�ٶ�ʸ����ǰ������xyzƽ�ƣ���������xyz��ת
///�Ի����˼�����ϵΪ��׼
EcBoolean EcCytonCommands::SetEEVelocity(VectorXd & EEVelo)
{
	// ���ȱ������ÿ���ģʽΪ�ٶȿ���ģʽ
	EcBoolean retVal = setControlMode(1);

	if (retVal)
	{
		//�����ٶ����ޣ�Ϊ�˶������ٶ�����
		VectorXd VeloLimit(6);
		VeloLimit << 0.3, 0.3, 0.3, 0.2, 0.2, 0.2;
		for (int j = 0; j < 6; ++j)
		{
			if (fabs(EEVelo(j))>VeloLimit(j))
			{
				EEVelo(j) = VeloLimit(j)*(EEVelo(j) / fabs(EEVelo(j)));//�������ٶ�������ȡ���ޣ����ű��ֲ���
			}
		}
		//��EEvelo��Eigen���ݸ�ʽת��ΪEcRealVector
		EcRealVector endVelo(6);
		for (int i = 0; i < 6; ++i)
		{
			endVelo[i] = EEVelo(i);
		}

		retVal = setDesiredVelocity(endVelo);

		// ���ó���ʱ�䣬��ʱ���ͣ������ʵ�ʿ���ʱ��������ѡ��
		EcSLEEPMS(60);
		//EcRealVector zeroVelo(endVelo.size());
		//retVal = setDesiredVelocity(zeroVelo);

		// change back to position control
		retVal = setControlMode(0);
	}
	return retVal;
}

///λ�˿���ģʽ�µĻ����������˶����ƣ���Ҫ������̽���˶�����ȡÿһ����̽������̬�仯����taskfunction�ı仯�����ȡ�ſ˱Ⱦ���ĳ�ֵ,
///����Ϊ��ǰ��Ҫ���Ƶ����ɶ���š���ǰ�����Ƶ����ɶ��˶����ٶ�,����ֵΪ��Ҫȡֵ��λ�˱仯����
VectorXd EcCytonCommands::GetEEPoseIncrement(int DoFNum, EcReal DoFVelo)
{
	EcVector Position_old, Position;
	EcVector Axis_old, Axis;
	EcReal Angle_old, Angle;
	EcOrientation Orientation_old, Orientation_New;
	//�Ȼ�ȡ�˶�ǰǰλ��
	GetPose(Position_old, Axis_old, Angle_old);
	//����̬��ǹ�ϵ��ȡ��̬
	Orientation_old.setFromAngleAxis(Angle_old, Axis_old);
	//�����˶���ÿ���˶�һ�����ɶ��˶�
	EcRealVector dv(6);
	dv[(DoFNum - 1)] = DoFVelo;//�����ٶȣ�ֻ��ָ�����ɶ��ٶȣ��������ɶ�Ϊ0
	EcBoolean retVal;
	retVal = setControlMode(1);//�����ٶȿ���ģʽ
	retVal = setDesiredVelocity(dv);
	EcSLEEPMS(1000);// ���ó���ʱ��1s
	EcRealVector zeroVelo(dv.size());
	retVal = setDesiredVelocity(zeroVelo);
	retVal = setControlMode(0);//����ģʽ���û�λ�ÿ���

	//��ȡ�˶���λ��
	GetPose(Position, Axis, Angle);
	Orientation_New.setFromAngleAxis(Angle, Axis);
	//��ȡ�����̬�仯
	Orientation_old.angleAxisBetween(Orientation_New, Angle, Axis);//����ֱ����Axis*Angle - Axis_old*Angle_old����Ҫʹ��ecOrientation��angleaxisbetween������������̬֮������
	VectorXd  d_Pose(6);
	d_Pose << (Position.x() - Position_old.x()), (Position.y() - Position_old.y()), (Position.z() - Position_old.z()),
		(Axis.x()*Angle), (Axis.y()*Angle), (Axis.z()*Angle);
	return d_Pose;
}

VectorXd EcCytonCommands::GetEEPoseIncrement(VectorXd Pose_old, VectorXd Pose_new)
{
	//��ȡԭʼ����
	EcVector Position_old(Pose_old(0), Pose_old(1), Pose_old(2));
	EcVector Position(Pose_new(0), Pose_new(1), Pose_new(2));
	EcVector Axis_old(Pose_old(3), Pose_old(4), Pose_old(5));
	EcVector Axis(Pose_new(3), Pose_new(4), Pose_new(5));
	EcReal Angle_old = Pose_old(6);
	EcReal Angle = Pose_new(6);
	//��pose�����л�ȡ��̬��Ϣ
	EcOrientation Orientation_old, Orientation_New;
	Orientation_old.setFromAngleAxis(Angle_old, Axis_old);
	Orientation_New.setFromAngleAxis(Angle, Axis);
	//��ȡ��̬�仯
	Orientation_old.angleAxisBetween(Orientation_New, Angle, Axis);//����ֱ����Axis*Angle - Axis_old*Angle_old����Ҫʹ��ecOrientation��angleaxisbetween������������̬֮������
	//��λ�˱仯��ֵ
	VectorXd  d_Pose(6);
	d_Pose << (Position.x() - Position_old.x()), (Position.y() - Position_old.y()), (Position.z() - Position_old.z()),
		(Axis.x()*Angle), (Axis.y()*Angle), (Axis.z()*Angle);
	return d_Pose;
}

EcBoolean EcCytonCommands::Homing()
{
	EcRealVector joints(7);
	for (int i = 0; i < 7; ++i)
	{
		joints[i] = 0.0;
	}
	EcBoolean retVal = setEndEffectorSet(JOINT_CONTROL_EE_SET);
	retVal = setJointValues(joints);
	retVal = setEndEffectorSet(FRAME_EE_SET);//ĩ��ִ��������ģʽ���û�FRAMEģʽ�����������ݻ�ȡ���б���
	return retVal;
}

void EcCytonCommands::VelocityMoving(int DoFNum, EcReal DoFVelo, EcU32 ms)
{
	setEndEffectorSet(FRAME_EE_SET);//��������ϵSET����֤�ܹ���ȡλ��
	EcRealVector dv(6);
	dv[(DoFNum - 1)] = DoFVelo;//�����ٶȣ�ֻ��ָ�����ɶ��ٶȣ��������ɶ�Ϊ0
	EcBoolean retVal;
	retVal = setControlMode(1);//�����ٶȿ���ģʽ
	retVal = setDesiredVelocity(dv);
	EcSLEEPMS(1000);// ���ó���ʱ��1s
	EcRealVector zeroVelo(dv.size());
	retVal = setDesiredVelocity(zeroVelo);
	retVal = setControlMode(0);//����ģʽ���û�λ�ÿ���
}

void EcCytonCommands::changeToFrameEE(EcCoordinateSystemTransformation &desiredPose)
{
	setEndEffectorSet(FRAME_EE_SET);//��������ϵSET����֤�ܹ���ȡλ��
	EcSLEEPMS(100);
	EcManipulatorEndEffectorPlacement actualEEPlacement;
	getActualPlacement(actualEEPlacement);
	desiredPose = actualEEPlacement.offsetTransformations()[0].coordSysXForm();
}