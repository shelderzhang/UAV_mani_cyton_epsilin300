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

   setEndEffectorSet(POINT_EE_SET); // point end effector set index
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
   const EcCoordinateSystemTransformation& pose
   )const
{

   std::cout<<"Desired pose:  x: "<<pose.translation().x()<< " y: " <<pose.translation().y()<<" z: " <<pose.translation().z()<<std::endl;

   setEndEffectorSet(FRAME_EE_SET); // frame end effector set index
   EcEndEffectorPlacement desiredPlacement(pose);
   EcManipulatorEndEffectorPlacement actualEEPlacement;
   EcCoordinateSystemTransformation offset, zero, actualCoord;
   zero.setTranslation(EcVector(0,0,0));

   getActualPlacement(actualEEPlacement);
   EcEndEffectorPlacementVector state = actualEEPlacement.offsetTransformations();
   if (state.size() < 1)
   {
      return EcFalse;
   }
   state[0]=desiredPlacement;
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
         EcPrint(Debug)<<"Achieved Pose"<<std::endl;
         achieved = EcTrue;
      }
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

//------------------file-----------manipulation action test-------------------------
EcBoolean EcCytonCommands::manipulationActionTest
   (
   const EcString& filename,
   const EcString& actionName
   )const
{
   EcManipulationActionManager actionManager;
   EcBoolean retVal = EcXmlObjectReaderWriter::readFromFile(actionManager, filename);
   
   if(!retVal)
   {
      std::cout << "Failed to load file : " << filename << std::endl;
      return retVal;
   }

   retVal = setManipulationActionManager(actionManager);

   EcManipulationScript script;
   setManipulationScript(script);

   if(!retVal)
   {
      return retVal;
   }

   retVal = setManipulationAction(actionName);
   if(!retVal)
   {
      return retVal;
   }

   retVal = startManipulation();
   if(!retVal)
   {
      return retVal;
   }

   Wait wait;
   setManipulationCompletedCallback(boost::bind(&Wait::actionExecCompletionCallback, &wait, _1, _2));

   // wait
   retVal = wait.waitForCompletion();

   return retVal;
}

//-----------------file------------manipulation action  series test-------------------------
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

//----------------file-------------manipulation director test-------------------------
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

//-----------------------------pick and place example-------------------------

EcBoolean EcCytonCommands::pickAndPlaceExample
   (
   const EcString& cytonModel
   )const
{
   EcRealVector initJoints(7);//vector of EcReals that holds the set of joint angles
   initJoints[1]=-.7;
   initJoints[3]=-.7;
   initJoints[5]=-.7;

   //Move Joints
   MoveJointsExample(initJoints,.000001);

   //open the gripper
   if(cytonModel == "1500" || cytonModel == "300" )
   {
      moveGripperExample(0.0078);
   }
   if(cytonModel == "1500R2" || cytonModel == "300PX" )
   {
      moveGripperExample(0.0149);
   }

   if(cytonModel == "1500" || cytonModel == "1500R2")
   {

      EcCoordinateSystemTransformation desiredPose;
      desiredPose.setTranslation(EcVector(0,.35,.2));
      EcOrientation orient;
      orient.setFrom123Euler(0,0,EcPi/2);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.115,.35,.2));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.115,.35,.05));
      frameMovementExample(desiredPose); 

      //close the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(-0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.001);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(.115,.35,.2));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(0,.35,.2));
      frameMovementExample(desiredPose);   

      orient.setFrom123Euler(0,0,EcPi);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);
      frameMovementExample(desiredPose); 

      desiredPose.setTranslation(EcVector(0,.35,.05));
      frameMovementExample(desiredPose);   


      //Opem the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.0149);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(0,.35,.2));
      frameMovementExample(desiredPose);   
   }

   if(cytonModel=="300" || cytonModel=="300PX" )
   {
      EcCoordinateSystemTransformation desiredPose;
      desiredPose.setTranslation(EcVector(0,.2,.15));
      EcOrientation orient;
      orient.setFrom123Euler(0,0,EcPi/2);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);

      setEndEffectorSet(FRAME_EE_SET); // frame end effector set index

      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.1,.2,.15));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.1,.2,.05));
      frameMovementExample(desiredPose); 

      //close the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(-0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.001);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(.1,.2,.15));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(0,.2,.15));
      frameMovementExample(desiredPose);   

      orient.setFrom123Euler(0,0,EcPi);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);
      frameMovementExample(desiredPose); 

      desiredPose.setTranslation(EcVector(0,.2,.05));
      frameMovementExample(desiredPose);   


      //close the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.0149);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(0,.2,.15));
      frameMovementExample(desiredPose);   
   }
   return EcTrue;


}

//-----------------------------pick planning example-------------------------
EcBoolean EcCytonCommands::pathPlanningExample
   (
   const EcCoordinateSystemTransformation& pose
   )const
{
   EcBoolean retVal = EcTrue;
   // run this the first time
   std::cout << "Running path planning" << std::endl;

   EcManipulatorEndEffectorPlacement actualEEPlacement,desiredEEPlacement;
   //switch to frame ee set, so the link doesnt move when we try and grip
   setEndEffectorSet(FRAME_EE_SET);
   EcSLEEPMS(100);
   //get the current placement of the end effectors
   getActualPlacement(actualEEPlacement);

   desiredEEPlacement = actualEEPlacement;

   if (desiredEEPlacement.offsetTransformations().size() < 1)
   {
      return EcFalse;
   }
   desiredEEPlacement.offsetTransformations()[0].setCoordSysXForm(pose);
   retVal = setPathPlanningDesiredPlacement(desiredEEPlacement);

   if(!retVal)
   {
      return EcFalse;
   }

   Wait wait;
   setManipulationCompletedCallback(boost::bind(&Wait::actionExecCompletionCallback, &wait, _1, _2));

   // wait
   retVal = wait.waitForCompletion();

   return retVal;
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
