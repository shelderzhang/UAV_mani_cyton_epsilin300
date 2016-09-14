//------------------------------------------------------------------------------
// Copyright (c) 2013 Energid Technologies. All rights reserved.
//
/// @file main.cpp
/// @brief Remote commands tester
//
//------------------------------------------------------------------------------
#include "ecCytonCommands.h"
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <iostream>

//#include <boost/assign/list_of.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/program_options/options_description.hpp>
//#include <boost/program_options/parsers.hpp>
//#include <boost/program_options/variables_map.hpp>

//namespace bpo = boost::program_options;

////------------------------------------------------------------------------------
#define RC_CHECK(fun) do \
   { \
      std::cout << "Calling " << #fun << std::endl; \
      if(!fun) \
      { \
         std::cerr << "Problem with command " << #fun << "\n"; \
      } \
      else \
      {\
        EcPrint(None) << #fun << " successfully completed!" << "\n"; \
      }\
   } while(0)

int main
(
int argc,
char **argv
)
{
 /*use the default ip, cytonVersion and performExample */
   //autor:gyzhang
   EcString ipAddress = "127.0.0.1";
   EcString cytonVersion = "300";
   EcU32 performExample = 3;
   EcCytonCommands cytonCommands;

   EcString cytonDir = Ec::Application::getDataDirectory("cyton");
   if (cytonDir.empty())
   {
      cytonDir = ".";
   }
   cytonCommands.openNetwork(ipAddress);
   EcVector endeffector_position_d;
   EcCoordinateSystemTransformation desiredPose;

   desiredPose.setTranslation(endeffector_position_d);
   EcOrientation orient;
   orient.setFrom123Euler(.4, -.2, .9);//set roll, pitch,yaw
   desiredPose.setOrientation(orient);
   RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

   if (performExample == 1)
   {
      EcRealVector jointposition;
      jointposition.resize(7);

      //moves to vertical position
      RC_CHECK(cytonCommands.MoveJointsExample(jointposition, .000001));

      jointposition[1] = -.7;
      jointposition[3] = -.7;
      jointposition[5] = -.7;

      //moves to forward position
      RC_CHECK(cytonCommands.MoveJointsExample(jointposition, .000001));
   }

   if (performExample == 2)
   {
      //open the gripper first
      if (cytonVersion == "1500" || cytonVersion == "300")
      {
         cytonCommands.moveGripperExample(0.0078);
      }
      if (cytonVersion == "1500R2" || cytonVersion == "300PX")
      {
         cytonCommands.moveGripperExample(0.0149);
      }

      EcCoordinateSystemTransformation desiredPose;
      desiredPose.setTranslation(EcVector(.2, .2, .3));
      RC_CHECK(cytonCommands.pointMovementExample(desiredPose));

      desiredPose.setTranslation(EcVector(-.2, .2, .3));
      RC_CHECK(cytonCommands.pointMovementExample(desiredPose));

      desiredPose.setTranslation(EcVector(0, .2, .3));
      RC_CHECK(cytonCommands.pointMovementExample(desiredPose));

      desiredPose.setTranslation(EcVector(0, .2, .1));
      RC_CHECK(cytonCommands.pointMovementExample(desiredPose));
   }

   if (performExample == 3)
   {
      //open the gripper first
      if (cytonVersion == "1500" || cytonVersion == "300")
      {
         cytonCommands.moveGripperExample(0.0078);
      }
      if (cytonVersion == "1500R2" || cytonVersion == "300PX")
      {
         cytonCommands.moveGripperExample(0.0149);
      }
      EcSLEEPMS(500);
      EcRealVector jointposition(7);
      jointposition[1] = -.7;
      jointposition[3] = -.7;
      jointposition[5] = -.7;


      //moves to forward position
      RC_CHECK(cytonCommands.MoveJointsExample(jointposition, .000001));//Joint Movement Example

      if (cytonVersion == "1500" || cytonVersion == "1500R2")
      {
         EcCoordinateSystemTransformation desiredPose;
         desiredPose.setTranslation(EcVector(.2, .25, .25));
         EcOrientation orient;
         orient.setFrom123Euler(.4, -.2, .9);//set roll, pitch,yaw
         desiredPose.setOrientation(orient);
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         desiredPose.setTranslation(EcVector(.2, .2, .1));
         orient.setFrom123Euler(0, 0, -EcPi);
         desiredPose.setOrientation(orient);
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         desiredPose.setTranslation(EcVector(-.2, .2, .1));
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         orient.setFrom123Euler(0, -EcPi / 6, -EcPi);
         desiredPose.setOrientation(orient);
         desiredPose.setTranslation(EcVector(0, .2, .1));
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         //move relative to last pose along -x axis
         EcCoordinateSystemTransformation relativetrans(EcVector(-.1, 0, 0));
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose * relativetrans));
      }
      if (cytonVersion == "300" || cytonVersion == "300PX")
      {
         EcCoordinateSystemTransformation desiredPose;
         desiredPose.setTranslation(EcVector(.1, .15, .15));
         EcOrientation orient;
         orient.setFrom123Euler(.4, -.2, .9);//set roll, pitch,yaw
         desiredPose.setOrientation(orient);
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         desiredPose.setTranslation(EcVector(.1, .2, .05));
         orient.setFrom123Euler(0, 0, -EcPi);
         desiredPose.setOrientation(orient);
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         desiredPose.setTranslation(EcVector(.1, .2, .05));
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         orient.setFrom123Euler(0, -EcPi / 6, -EcPi);
         desiredPose.setOrientation(orient);
         desiredPose.setTranslation(EcVector(0, .2, .05));
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose));

         //move relative to last pose along -x axis
         EcCoordinateSystemTransformation relativetrans(EcVector(-.15, 0, 0));
         RC_CHECK(cytonCommands.frameMovementExample(desiredPose * relativetrans));
      }


   }

   if (performExample == 4)
   {
      //gripper example
      if (cytonVersion == "300" || cytonVersion == "1500")
      {
         cytonCommands.moveGripperExample(.0079);
         cytonCommands.moveGripperExample(-.0079);
      }
      //with AX gripper
      if (cytonVersion == "300PX" || cytonVersion == "1500R2")
      {
         cytonCommands.moveGripperExample(.0001);
         cytonCommands.moveGripperExample(.0149);
      }
   }

   if (performExample == 5)
   {
      //manipulation action example
      EcString file = "testRemoteCommandActions.xml";
      EcString action = "Action 1";
      RC_CHECK(cytonCommands.manipulationActionTest(file, action));
   }

   if (performExample == 6)
   {
      //manipulation action example
      EcString file = "testRemoteCommandActions.xml";
      RC_CHECK(cytonCommands.manipulationActionSeriesTest(file));
   }

   if (performExample == 7)
   {
      //Pick and place example
      RC_CHECK(cytonCommands.pickAndPlaceExample(cytonVersion));
   }

   if (performExample == 8)
   {
      //Path planning example
      EcCoordinateSystemTransformation desiredPose;
      desiredPose.setTranslation(EcVector(.21, .15, .15));
      EcOrientation orient;
      orient.setFrom321Euler(1, -.3, -.6);//set yaw, pitch,roll
      desiredPose.setOrientation(orient);
      RC_CHECK(cytonCommands.pathPlanningExample(desiredPose));
   }

   if (performExample == 9)
   {
      //velocity control example
      EcRealVector endVelo(3);
      //x direction, 20cm/s
      endVelo[0] = 0.2;

      RC_CHECK(cytonCommands.endEffectorVelocityTest(endVelo));
   }

   if (performExample == 10)
   {
      //manipulation director example
      EcString file = "Example Cyton Direction.ecd";
      RC_CHECK(cytonCommands.manipulationDirectorTest(file));
   }

   cytonCommands.closeNetwork();

   return 1;
}
