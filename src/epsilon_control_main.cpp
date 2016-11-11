/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <foundCommon/ecCoordSysXForm.h>

#include "epsilon_control_main.h"
//#include "ecCytonCommands.h"

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

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments

    char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);

	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

    /*
     * Construct a Serial_Port object
     * This object handles the opening and closing of the Robai cyton epsilon300 controller
     *  computer's serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock (int fd).
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
     * Construct an Autopilot_Interface objiect,This object will start
     *  two threads for read and write MAVlink message of
     * commands to move the robotic arm and its status.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);

    /*
     *Construct an EcCytonCommands  object that control the robotic
     * arm and get the status of the arm
     *
     */
    EcCytonCommands cytonCommands(&autopilot_interface);

	/*
	 * Setup interrupt signal handler
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to close threads and the port.
     *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
    cytonCommands_quit = &cytonCommands;
	signal(SIGINT,quit_handler);


    // --------------------------------------------------------------------------
    //   Cyton epsilon300 control start
    // --------------------------------------------------------------------------

//   EcString cytonVersion = "300PX";
//    EcString cytonDir = Ec::Application::getDataDirectory("cyton");
//    if (cytonDir.empty())
//    {
//       cytonDir = ".";
//    }
    EcString ipAddress = "127.0.0.1";
    cytonCommands.openNetwork(ipAddress);

   // initialize robotic arm
    EcSLEEPMS(2000);
    /*
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();
    autopilot_interface.start();
    cytonCommands.start();

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
//       cytonCommands.moveGripperExample(.0001);
       EcSLEEPMS(500);
/*       cytonCommands.moveGripperExample(.0149)*/;
       EcSLEEPMS(500);
    RC_CHECK(cytonCommands.MoveJointsExample(jointposition, .000001));//Joint Movement Example
//    cytonCommands.moveGripperExample(.0001);
    EcSLEEPMS(500);
//    cytonCommands.moveGripperExample(.0149);
    EcSLEEPMS(500);


//    /*Copy the target end-effector frame to Cyton epsilon300*/

    while(1)
    {
    cytonCommands.setTargFrame();
    printf("target point:\n%f;  %f;  %f;\n",cytonCommands.targFrame.x,cytonCommands.targFrame.y,cytonCommands.targFrame.z);
    cytonCommands.frameMovementExample();
//    cytonCommands.moveGripperExample(.0001);
//    cytonCommands.moveGripperExample(.0149);
    }
    // --------------------------------------------------------------------------
    //   Join threads of serial port
    // --------------------------------------------------------------------------
    pthread_join (autopilot_interface.read_tid, NULL);
    pthread_join (autopilot_interface.write_tid, NULL);
    pthread_join (cytonCommands.read_armstatus_tid, NULL);
    cytonCommands.closeNetwork();

	return 0;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
    printf("Terminating at user requst\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

    // serial port
    try {

        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

 // serial port
    try {

        cytonCommands_quit->handle_quit(sig);
    }
    catch (int error){}




	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
        fprintf(stderr,"epsilon_control threw exception %i \n" , error);
		return error;
	}

}


