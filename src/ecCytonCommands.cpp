//------------------------------------------------------------------------------
// Copyright (c) 2004-2013 Energid Technologies. All rights reserved.
//
/// @file ecCytonCommands.cpp
//
//------------------------------------------------------------------------------
#include "ecCytonCommands.h"
#include <control/ecEndEffectorSet.h>
#include <controlCore/ecFrameEndEffector.h>
#include <control/ecManipEndEffectorPlace.h>
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <manipulationDirector/ecManipulationActionManager.h>
#include <manipulationDirector/ecManipulationScript.h>
#include <manipulationDirector/ecManipulationDirector.h>
#include <math.h>
#include <remoteCommand/ecRemoteCommand.h>
#include <xmlReaderWriter/ecXmlObjectReaderWriter.h>
#include <iostream>
#include <boost/bind.hpp>
#include <stdio.h>

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
         m_CompletionReceived    = EcTrue;
         m_Success               = success;
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
EcCytonCommands::EcCytonCommands
   (Autopilot_Interface *autopilot_interface_
   )
{
    autopilot_interface = autopilot_interface_;
    memset(&frameStatus, 0, sizeof(frameStatus));
    memset(&joinStatus, 0, sizeof(joinStatus));
    memset(&targFrame, 0, sizeof(targFrame));
}
//----------------------------------destructor---------------------------------
EcCytonCommands::~EcCytonCommands
   (
   )
{
}

//----------------------------------overloading = -----------------------------
 EcCytonCommands& EcCytonCommands:: operator=
   (
    EcCytonCommands& orig
   )
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
   if(!init(m_IpAddress))
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
   EcBoolean retVal=EcTrue;
   setEndEffectorSet(JOINT_CONTROL_EE_SET);
   EcSLEEPMS(500);

   //vector of EcReals that holds the set of joint angles
   EcRealVector currentJoints;
   retVal &= getJointValues(currentJoints);

   size_t size = currentJoints.size();
   if(size < jointPosition.size())
   {
      size = currentJoints.size();
   }
   else if(size >= jointPosition.size())
   {
      size = jointPosition.size();
   }

   EcPrint(Debug) <<"Current Joint Angles: ( ";
   for(size_t ii=0; ii<size; ++ii)
   {
      EcPrint(Debug)  << currentJoints[ii] << "," ;
      currentJoints[ii] = jointPosition[ii];
   }
   EcPrint(Debug)  << " )" << std::endl;

   std::cout << "Desired joint Angles: ( ";
   for(size_t ii=0; ii<size; ++ii)
   {
      std::cout << currentJoints[ii] << "," ;
   }
   std::cout <<" )" << std::endl;

   retVal &= setJointValues(currentJoints);

   //Check if achieved
   EcBooleanVector jointAchieved;
   jointAchieved.resize(size);
   EcBoolean positionAchieved = EcFalse;
      
   // if it hasnt been achieved after 5 sec, return false
   EcU32 timeout = 5000;
   EcU32 interval = 10;
   EcU32 count = 0;

   while(!positionAchieved && !(count >= timeout/interval))
   {
      EcSLEEPMS(interval);
      count++;

      EcPrint(Debug) << "Moving ";
      getJointValues(currentJoints);
      EcPrint(Debug) << "Current Joints: ";
      for(size_t ii=0; ii<size; ++ii)
      {

         EcPrint(Debug)  << " , " << currentJoints[ii];

         if( std::abs(jointPosition[ii]-currentJoints[ii])<angletolerance)
         {
            jointAchieved[ii]=EcTrue;
         }
      }
      EcPrint(Debug) <<std::endl;
      for(size_t ii=0; ii<size; ++ii)
      {
         if(!jointAchieved[ii])
         {
            positionAchieved=EcFalse;
            break;
         }
         else
         {
            positionAchieved=EcTrue;
         }
      }  
   }
   EcPrint(Debug) << " Final Joint Angles: (";
   for(size_t ii=0; ii<size; ++ii)
   {
      EcPrint(Debug) <<  currentJoints[ii] << "," ;
   }
   EcPrint(Debug) <<" ) " << std::endl;

   
   std::cout<< (positionAchieved ? "Achieved Joint State" : "Failed to Achieve Joint State") <<std::endl;

   return positionAchieved;
}

//-----------------------------Point Movement Example-------------------------
EcBoolean EcCytonCommands::pointMovementExample
   (
const EcCoordinateSystemTransformation& pose
   )const
{

   std::cout<<"Desired pose:  x: "<<pose.translation().x()<< " y: " <<pose.translation().y()<<" z: " <<pose.translation().z()<<std::endl;

   setEndEffectorSet(FRAME_EE_SET); // frame end effector set index
   EcEndEffectorPlacement desiredPlacement(pose);
   EcManipulatorEndEffectorPlacement actualEEPlacement;
   EcCoordinateSystemTransformation offset, zero, actualCoord;
   zero.setTranslation(EcVector(0,0,0));

   //set the desired position
   setDesiredPlacement(desiredPlacement,0,0);

   // if it hasnt been achieved after 5 sec, return false
   EcU32 timeout = 5000;
   EcU32 interval = 10;
   EcU32 count = 0;
   EcBoolean achieved = EcFalse;
   while(!achieved && !(count >= timeout/interval))
   {
      EcSLEEPMS(interval);
      count++;

      EcPrint(Debug) << "Moving "<<std::endl;
      getActualPlacement(actualEEPlacement);
      if (actualEEPlacement.offsetTransformations().size() < 1)
      {
         return EcFalse;
      }


      actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
      
      //get the transformation between the actual and desired 
      offset=(actualCoord.inverse()) * pose;
      EcPrint(Debug)<<"distance between actual and desired: "<<offset.translation().mag()<<std::endl;

      if(offset.approxEq(zero,.00001))
      {
         achieved = EcTrue;
      }
      
   }
   std::cout<< (achieved ? "Achieved Pose" : "Failed to Achieve Pose") <<std::endl;
   return achieved;
}

//-----------------------------Frame Movement Example-------------------------
EcBoolean EcCytonCommands::frameMovementExample
   (

   )
{
   setEndEffectorSet(FRAME_EE_SET); // frame end effector set index

   // Copy the target end-effector frame to Cyton epsilon300 need lock ????

     setTargFrame();

   printf("target point:\n%f;  %f;  %f;\n",targFrame.x,targFrame.y,targFrame.z);

   /*translate end-effector frame to cyton desiredPose */
//   EcVector endeffector_position_d;
   EcCoordinateSystemTransformation desiredPose;

//   endeffector_position_d.setX(targFrame.x);
//   endeffector_position_d.setY(targFrame.y);
//   endeffector_position_d.setZ(targFrame.z);

//   desiredPose.setTranslation(EcVector(.1, .2, .05));
   desiredPose.setTranslation(EcVector(targFrame.x,targFrame.y,targFrame.z));
   EcOrientation orient;//set roll, pitch,yaw
   orient.setFrom123Euler( 0, 0,-EcPi);

 //  orient.setFrom321Euler(targFrame.yaw, targFrame.pitch, targFrame.roll);
   desiredPose.setOrientation(orient);

   /*define desiredPlacement to set epsilon300 move*/
   EcEndEffectorPlacement desiredPlacement(desiredPose);

   /*get status of epsilon300 arm*/
   EcManipulatorEndEffectorPlacement actualEEPlacement;
   EcRealVector currentJoints;
   EcCoordinateSystemTransformation offset, zero, actualCoord;
   zero.setTranslation(EcVector(0,0,0));

   getActualPlacement(actualEEPlacement);
    if (actualEEPlacement.offsetTransformations().size() < 1)
   {
      return EcFalse;
   }

   // if it hasnt been achieved after 5 sec, return false
   EcU32 timeout = 5000;
   EcU32 interval = 10;
   EcU32 count = 0;
   EcBoolean achieved = EcFalse;

   if (targFrame.arm_enable == 1)
//   if (1)
   {
     //set the desired position
     setDesiredPlacement(desiredPlacement,0,0);
   while(!achieved && !(count >= timeout/interval))
   {
      EcSLEEPMS(interval);
      count++;

      EcPrint(Debug) << "Moving "<<std::endl;
      getActualPlacement(actualEEPlacement);
      if (actualEEPlacement.offsetTransformations().size() < 1)
      {
         return EcFalse;
      }

      actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
      getFrameStatus(actualCoord);
      // lock Frame status
      updateFrameStatus();

      getJointValues(currentJoints);
      getJoinStatus(currentJoints);
      // lock Join status
      updateJoinStatus();

      //get the transformation between the actual and desired 
      offset=(actualCoord.inverse()) * desiredPose;
      EcPrint(Debug)<<"distance between actual and desired: "<<offset.translation().mag()<<std::endl;

      if(offset.approxEq(zero,.00001))
      {
         EcPrint(Debug)<<"Achieved Pose"<<std::endl;
         achieved = EcTrue;
      }
   }
   }
   else
   {
       getActualPlacement(actualEEPlacement);
       if (actualEEPlacement.offsetTransformations().size() < 1)
       {
          return EcFalse;
       }

       actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
       getFrameStatus(actualCoord);
       // lock Frame status
       updateFrameStatus();

       getJointValues(currentJoints);
       getJoinStatus(currentJoints);
       // lock Join status
       updateJoinStatus();
   }
   std::cout<< (achieved ? "Achieved Pose" : "Failed to Achieve Pose") <<std::endl;
   return achieved;

}

//-----------------------------move gripper test-------------------------
EcBoolean EcCytonCommands::moveGripperExample
   (
   const EcReal gripperPos
   )const
{
   EcManipulatorEndEffectorPlacement actualEEPlacement,desiredEEPlacement;
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
   gripperfinger1trans.setTranslation(EcVector(0,0,gripperPos));
   EcEndEffectorPlacement finger1placement = state[1];
   finger1placement.setCoordSysXForm(gripperfinger1trans);
   state[1]=finger1placement;

   desiredEEPlacement.setOffsetTransformations(state);

   //set the desired placement
   setDesiredPlacement(desiredEEPlacement,0);

   // if it hasnt been achieved after 2 sec, return false
   EcU32 timeout = 2000;
   EcU32 interval = 10;
   EcU32 count = 0;
   EcBoolean achieved = EcFalse;
   while(!achieved && !(count >= timeout/interval))
   {
      EcSLEEPMS(interval);
      count++;

      EcPrint(Debug) << "Moving "<<std::endl;

      getActualPlacement(actualEEPlacement);
      EcEndEffectorPlacementVector currentState = actualEEPlacement.offsetTransformations();
      EcCoordinateSystemTransformation gripperfinger1trans = currentState[1].coordSysXForm();
      EcReal difference = std::abs(gripperPos - gripperfinger1trans.translation().z());
      EcPrint(Debug)<<"distance between actual and desired: "<< difference <<std::endl;

      if(difference < .000001)
      {
         achieved = EcTrue;
      }
   }
   std::cout<< (achieved ? "Achieved Gripper Position" : "Failed to Achieve Gripper Position") <<std::endl;
   return achieved;
}

//-----------------------------end effector velocity test-------------------------
EcBoolean EcCytonCommands::endEffectorVelocityTest
   (
   const EcRealVector &endVelo
   ) const
{
   // change to velocity control
   EcBoolean retVal = setControlMode(1);
   if(retVal)
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


//-----------------------------hardware enable test-------------------------
EcBoolean EcCytonCommands::hardwareEnableTest
   (
   const EcBoolean flag
   )const
{
   return setHardwareEnable(flag);
}

//-----------------------------reset to home-------------------------
EcBoolean EcCytonCommands::resetToHome
   (
   )const
{
   EcRealVector joints;
   EcBoolean retVal = getJointValues(joints);

   size_t size = joints.size();

   // increment all joints except the last
   for(size_t ii=0; ii<size; ++ii)
   {
      joints[ii] = 0.0;
   }

   retVal &= setJointValues(joints);
   EcSLEEPMS(2000);

   return retVal;
}

EcBoolean EcCytonCommands::serialComTest
   (
   )const
{
    autopilot_interface->endeff_frame_status.x = 1;
    autopilot_interface->endeff_frame_status.y = 2;
    autopilot_interface->endeff_frame_status.z = 3;
    autopilot_interface->endeff_frame_status.roll = 4;
    autopilot_interface->endeff_frame_status.pitch = 5;
    autopilot_interface->endeff_frame_status.yaw = 6;
    autopilot_interface->endeff_frame_status.vx=7;
    autopilot_interface->endeff_frame_status.vy=8;
    autopilot_interface->endeff_frame_status.vz=9;
    autopilot_interface->endeff_frame_status.roll_rate =10;
    autopilot_interface->endeff_frame_status.pitch_rate =11;
    autopilot_interface->endeff_frame_status.yaw_rate =12;
    autopilot_interface->endeff_frame_status.arm_enable=1;
    autopilot_interface->endeff_frame_status.gripper_posi=2024;
    autopilot_interface->endeff_frame_status.gripper_status=1;

    autopilot_interface->mani_joints.joint_posi_1 = 7;
    autopilot_interface->mani_joints.joint_posi_2 = 8;
    autopilot_interface->mani_joints.joint_posi_3 = 9;
    autopilot_interface->mani_joints.joint_posi_4 = 10;
    autopilot_interface->mani_joints.joint_posi_5 = 11;
    autopilot_interface->mani_joints.joint_posi_6 = 12;
    autopilot_interface->mani_joints.joint_posi_7 = 13;

    autopilot_interface->mani_joints.joint_rate_1 =1;
    autopilot_interface->mani_joints.joint_rate_2 =2;
    autopilot_interface->mani_joints.joint_rate_3 =3;
    autopilot_interface->mani_joints.joint_rate_4 =4;
    autopilot_interface->mani_joints.joint_rate_5 =5;
    autopilot_interface->mani_joints.joint_rate_6 =6;
    autopilot_interface->mani_joints.joint_rate_7 =7;

    autopilot_interface->mani_joints.torque_1 = 7;
    autopilot_interface->mani_joints.torque_2 = 6;
    autopilot_interface->mani_joints.torque_3 = 5;
    autopilot_interface->mani_joints.torque_4 = 4;
    autopilot_interface->mani_joints.torque_5 = 3;
    autopilot_interface->mani_joints.torque_6 = 2;
    autopilot_interface->mani_joints.torque_7 = 1;

   return 0;
}

EcBoolean EcCytonCommands::setTargFrame()
{
targFrame.x = autopilot_interface->target_endeff_frame.x;
targFrame.y = autopilot_interface->target_endeff_frame.y;
targFrame.z = autopilot_interface->target_endeff_frame.z;
targFrame.vx = autopilot_interface->target_endeff_frame.vx;
targFrame.vy = autopilot_interface->target_endeff_frame.vy;
targFrame.vz = autopilot_interface->target_endeff_frame.vz;
targFrame.roll = autopilot_interface->target_endeff_frame.roll;
targFrame.pitch = autopilot_interface->target_endeff_frame.pitch;
targFrame.yaw = autopilot_interface->target_endeff_frame.yaw;
targFrame.roll_rate = autopilot_interface->target_endeff_frame.roll_rate;
targFrame.pitch_rate = autopilot_interface->target_endeff_frame.pitch_rate;
targFrame.yaw_rate = autopilot_interface->target_endeff_frame.yaw_rate;
targFrame.arm_enable = autopilot_interface->target_endeff_frame.arm_enable;
return 0;
}

EcBoolean EcCytonCommands::getFrameStatus
(EcCoordinateSystemTransformation coordStatus)
{
    EcReal current_roll, current_pitch, current_yaw;
    frameStatus.x = coordStatus.translation().x();
    frameStatus.y = coordStatus.translation().y();
    frameStatus.z = coordStatus.translation().z();
    coordStatus.orientation().get321Euler(current_yaw,current_pitch,current_roll);
    frameStatus.roll = current_roll;
    frameStatus.pitch = current_pitch;
    frameStatus.yaw = current_yaw;
    return 0;
}
EcBoolean EcCytonCommands::getJoinStatus
(EcRealVector currJoin)

{
    joinStatus.joint_posi_1 = currJoin[0];
    joinStatus.joint_posi_2 = currJoin[1];
    joinStatus.joint_posi_3 = currJoin[2];
    joinStatus.joint_posi_4 = currJoin[3];
    joinStatus.joint_posi_5 = currJoin[4];
    joinStatus.joint_posi_6 = currJoin[5];
    joinStatus.joint_posi_7 = currJoin[6];
    return 0;
}

EcBoolean EcCytonCommands::updateFrameStatus()
{
    autopilot_interface->endeff_frame_status.x = frameStatus.x;
    autopilot_interface->endeff_frame_status.y = frameStatus.y;
    autopilot_interface->endeff_frame_status.z = frameStatus.z;
    autopilot_interface->endeff_frame_status.roll = frameStatus.roll;
    autopilot_interface->endeff_frame_status.pitch = frameStatus.pitch;
    autopilot_interface->endeff_frame_status.yaw = frameStatus.yaw;
    autopilot_interface->endeff_frame_status.vx = frameStatus.vx;
    autopilot_interface->endeff_frame_status.vy = frameStatus.vy;
    autopilot_interface->endeff_frame_status.vz = frameStatus.vz;
    autopilot_interface->endeff_frame_status.roll_rate = frameStatus.roll_rate;
    autopilot_interface->endeff_frame_status.pitch_rate = frameStatus.pitch_rate;
    autopilot_interface->endeff_frame_status.yaw_rate = frameStatus.yaw_rate;
    autopilot_interface->endeff_frame_status.arm_enable = frameStatus.arm_enable;
    autopilot_interface->endeff_frame_status.gripper_posi = frameStatus.gripper_posi;
    autopilot_interface->endeff_frame_status.gripper_status = frameStatus.gripper_status;
    return 0;
}

EcBoolean EcCytonCommands:: updateJoinStatus()
{
    autopilot_interface->mani_joints.joint_posi_1 = joinStatus.joint_posi_1;
    autopilot_interface->mani_joints.joint_posi_2 = joinStatus.joint_posi_2;
    autopilot_interface->mani_joints.joint_posi_3 = joinStatus.joint_posi_3;
    autopilot_interface->mani_joints.joint_posi_4 = joinStatus.joint_posi_4;
    autopilot_interface->mani_joints.joint_posi_5 = joinStatus.joint_posi_5;
    autopilot_interface->mani_joints.joint_posi_6 = joinStatus.joint_posi_6;
    autopilot_interface->mani_joints.joint_posi_7 = joinStatus.joint_posi_7;

    autopilot_interface->mani_joints.joint_rate_1 = joinStatus.joint_rate_1;
    autopilot_interface->mani_joints.joint_rate_2 = joinStatus.joint_rate_2;
    autopilot_interface->mani_joints.joint_rate_3 = joinStatus.joint_rate_3;
    autopilot_interface->mani_joints.joint_rate_4 = joinStatus.joint_rate_4;
    autopilot_interface->mani_joints.joint_rate_5 = joinStatus.joint_rate_5;
    autopilot_interface->mani_joints.joint_rate_6 = joinStatus.joint_rate_6;
    autopilot_interface->mani_joints.joint_rate_7 = joinStatus.joint_rate_7;

    autopilot_interface->mani_joints.torque_1 = joinStatus.torque_1;
    autopilot_interface->mani_joints.torque_2 = joinStatus.torque_2;
    autopilot_interface->mani_joints.torque_3 = joinStatus.torque_3;
    autopilot_interface->mani_joints.torque_4 = joinStatus.torque_4;
    autopilot_interface->mani_joints.torque_5 = joinStatus.torque_5;
    autopilot_interface->mani_joints.torque_6 = joinStatus.torque_6;
    autopilot_interface->mani_joints.torque_7 = joinStatus.torque_7;
    return 0;
}
