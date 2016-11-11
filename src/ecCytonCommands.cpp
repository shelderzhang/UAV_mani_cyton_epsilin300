//------------------------------------------------------------------------------
// Copyright (c) 2004-2013 Energid Technologies. All rights reserved.
//
/// @file ecCytonCommands.cpp
//
//------------------------------------------------------------------------------
#include "ecCytonCommands.h"
#include <control/ecEndEffectorSet.h>
#include <controlCore/ecFrameEndEffector.h>
//#include <control/ecManipEndEffectorPlace.h>
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


void*
read_armstatus_thread(void *args)
{
    // takes an autopilot object argument
    EcCytonCommands *eccytoncommands = (EcCytonCommands *)args;

    // run the object's read thread
    eccytoncommands->read_armstatus_thread_main();


    return NULL;
}
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
EcCytonCommands::EcCytonCommands ( )
{
}
EcCytonCommands::EcCytonCommands
   (Autopilot_Interface *autopilot_interface_)
{
    autopilot_interface = autopilot_interface_;
    memset(&frameStatus, 0, sizeof(frameStatus));
    memset(&joinStatus, 0, sizeof(joinStatus));
    memset(&targFrame, 0, sizeof(targFrame));
    memset(&actualEEPlacement, 0, sizeof(actualEEPlacement));
    memset(&currentJoints, 0, sizeof(currentJoints));

    read_armstatus_tid  = 0; // read thread id
    time_to_exit   = false;  // flag to signal thread exit
    reading_status = 0;
    int result = pthread_mutex_init(&actualEEP_lock, NULL);
    if ( result != 0 )
    {
        printf("\n ecCytoncommand mutex init failed\n");
        throw 1;
    }
    pthread_mutex_init(&actual_joint_lock, NULL);
}
//----------------------------------destructor---------------------------------
EcCytonCommands::~EcCytonCommands ( )
{
    pthread_mutex_destroy(&actualEEP_lock);
    pthread_mutex_destroy(&actual_joint_lock);
}

//----------------------------------overloading = -----------------------------
 EcCytonCommands& EcCytonCommands:: operator=
   (EcCytonCommands& orig)
{
   return *this;
}

//----------------------------------overloading == ----------------------------
EcBoolean EcCytonCommands:: operator==
   (const EcCytonCommands& orig)const
{
   return EcTrue;
}

//----------------------------------open network-------------------------------
EcBoolean EcCytonCommands::openNetwork
   (const EcString& m_IpAddress)const
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
   ( )const
{
   shutdown();
   return EcTrue;
}

//----------------------------------joint commands test------------------------

EcBoolean EcCytonCommands::MoveJointsExample
   (
   const EcRealVector jointPosition,
   const EcReal angletolerance
   )
{
   EcBoolean retVal=EcTrue;
   setEndEffectorSet(JOINT_CONTROL_EE_SET);
   EcSLEEPMS(500);

   size_t size = jointPosition.size();
   retVal &= setJointValues(jointPosition);

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

      //check if every joint achieved
      pthread_mutex_lock(&actual_joint_lock);
      for(size_t ii=0; ii<size; ++ii)
      {
         if( std::abs(jointPosition[ii]-currentJoints[ii])<angletolerance)
         {
            jointAchieved[ii]=EcTrue;
         }
      }
      pthread_mutex_unlock(&actual_joint_lock);


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
EcBoolean EcCytonCommands::frameMovementExample ()
{
   setEndEffectorSet(FRAME_EE_SET); // frame end effector set index

   /*translate end-effector frame to cyton desiredPose */
   EcCoordinateSystemTransformation desiredPose;
   desiredPose.setTranslation(EcVector(targFrame.x,targFrame.y,targFrame.z));
   EcOrientation orient;//setyaw, pitch,roll
//   orient.setFrom123Euler( EcPi/6, -EcPi/6,EcPi/3);
   orient.setFrom321Euler(targFrame.yaw, targFrame.pitch, targFrame.roll);
   desiredPose.setOrientation(orient);

   /*define desiredPlacement to set epsilon300 move*/
   EcEndEffectorPlacement desiredPlacement(desiredPose);

   /*define the erro variable*/
   EcCoordinateSystemTransformation offset, zero, actualCoord;
   zero.setTranslation(EcVector(0,0,0));

  // if it hasnt been achieved after 5 sec, return false
   EcU32 timeout = 5000;
   EcU32 interval = 10;
   EcU32 count = 0;
   EcBoolean achieved = EcFalse;

     //set the desired position
     setDesiredPlacement(desiredPlacement,0,0);
   while(!achieved && !(count >= timeout/interval))
   {
      EcSLEEPMS(interval);
      count++;

      pthread_mutex_lock(&actualEEP_lock);
      actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
      pthread_mutex_unlock(&actualEEP_lock);

      //get the transformation between the actual and desired
      offset=(actualCoord.inverse()) * desiredPose;
      if(offset.approxEq(zero,.00001))
      {
         achieved = EcTrue;
      }
   }

   std::cout<< (achieved ? "Achieved Pose" : "Failed to Achieve Pose") <<std::endl;
   return achieved;

}

//-----------------------------move gripper test-------------------------
EcBoolean EcCytonCommands::moveGripperExample
   ( const EcReal gripperPos )
{

    EcManipulatorEndEffectorPlacement desiredEEPlacement;
    //switch to frame ee set, so the link doesnt move when we try and grip
    setEndEffectorSet(FRAME_EE_SET);
    EcSLEEPMS(100);
    //get the current placement of the end effectors
    pthread_mutex_lock(&actualEEP_lock);
    EcEndEffectorPlacementVector state = actualEEPlacement.offsetTransformations();
    pthread_mutex_unlock(&actualEEP_lock);

    //0 is the Wrist roll link (point or frame end effector),
    //1 is the first gripper finger link (linear constraint end effector)



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

       pthread_mutex_lock(&actualEEP_lock);
       EcEndEffectorPlacementVector currentState = actualEEPlacement.offsetTransformations();
       pthread_mutex_unlock(&actualEEP_lock);
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
   )
{
    EcBoolean retVal;
    EcRealVector jointposition(7);
       jointposition[1] = -0.7;
       jointposition[3] = -0.7;
       jointposition[5] = -0.7;

//       jointposition[0] = -1.6;
//       jointposition[1] = -0.5;
//       jointposition[2] = 0;
//       jointposition[3] = -0.5;
//       jointposition[4] = 0;
//       jointposition[5] = 0.47;
//       jointposition[6] = -1.6;
    // initialize robotic arm COM in center
//    jointposition[0] = -1.6;
//    jointposition[1] = 0.6;
//    jointposition[2] = 0;
//    jointposition[3] = -1.6;
//    jointposition[4] = -0.1;
//    jointposition[5] = 0.6;
//    jointposition[6] = -1.6;

    //moves to forward position

    MoveJointsExample(jointposition, .000001);//Joint Movement Example
    moveGripperExample(.0135);

   return retVal;
}

//-----------------------------read_armstatus_thread_main-------------------------
void EcCytonCommands::read_armstatus_thread_main()
{

   printf("\n epsilon300 read_armstatus_thread is running! \n");

    EcCoordinateSystemTransformation actualCoord;

while ( !time_to_exit )
{
    pthread_mutex_lock(&actualEEP_lock);
    getActualPlacement(actualEEPlacement);
    pthread_mutex_unlock(&actualEEP_lock);

    actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
    getFrameStatus(actualCoord);
    updateFrameStatus();

    pthread_mutex_lock(&actual_joint_lock);
    getJointValues(currentJoints);
    pthread_mutex_unlock(&actual_joint_lock);

    getJoinStatus(currentJoints);
    updateJoinStatus();
    usleep(1000);
}
}

//-----------------------------thread start-------------------------
void EcCytonCommands::start()
{ 

    if (pthread_create( &read_armstatus_tid, NULL, &read_armstatus_thread, this ))
    {
        printf("\n error:fail to create read_armstatus_thread \n");
    }
}


void EcCytonCommands::stop()
{

    printf("Close read_armstatus_thread \n");

    // signal exit
    time_to_exit = true;
    // wait for exit
    pthread_join(read_armstatus_tid ,NULL);

}


void EcCytonCommands::handle_quit( int sig )
{
    try {
        stop();
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop serial port\n");
    }
}

EcBoolean EcCytonCommands::setTargFrame()
{
pthread_mutex_lock(&(autopilot_interface->target_lock));
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
pthread_mutex_unlock(&(autopilot_interface->target_lock));
return 0;
}

EcBoolean EcCytonCommands::getFrameStatus
(EcCoordinateSystemTransformation coordStatus)
{
    EcReal current_roll, current_pitch, current_yaw;
    coordStatus.orientation().get321Euler(current_yaw,current_pitch,current_roll);

    frameStatus.x = coordStatus.translation().x();
    frameStatus.y = coordStatus.translation().y();
    frameStatus.z = coordStatus.translation().z();
    frameStatus.roll = current_roll;
    frameStatus.pitch = current_pitch;
    frameStatus.yaw = current_yaw;

//    printf("123euler:\n%f;  %f;  %f;\n",current_roll,current_pitch,current_yaw);

//    coordStatus.orientation().get321Euler(current_yaw1,current_pitch1,current_roll1);
//    printf("321euler:\n%f;  %f;  %f;\n",current_roll1,current_pitch1,current_yaw1);
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
    pthread_mutex_lock(&(autopilot_interface->endeff_lock));
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
    pthread_mutex_unlock(&(autopilot_interface->endeff_lock));
    return 0;
}

EcBoolean EcCytonCommands:: updateJoinStatus()
{
    pthread_mutex_lock(&(autopilot_interface->joints_lock));
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
    pthread_mutex_unlock(&(autopilot_interface->joints_lock));
    return 0;
}
