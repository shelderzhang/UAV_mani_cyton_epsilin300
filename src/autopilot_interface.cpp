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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
    system_id = 0;
    companion_id = 0;
    target_endeff_frame.arm_enable = 0;
    endeff_frame_status.arm_enable = 0;
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	// Blocking wait for new data
    printf("read_message\n");

    while (!time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------

        mavlink_message_t message;
        success = serial_port->read_message(message);
        // ---------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
              system_id = message.sysid;
              companion_id = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{
                case MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME:
                {
                    printf("MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME\n");
                    mavlink_msg_target_endeff_frame_decode(&message, &target_endeff_frame);
                    printf("target_endeff_frame :\n x= %f;  y = %f  \n",target_endeff_frame.x,target_endeff_frame.y);
                    break;
                }
				default:
				{
                    printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message



		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write end effector frame status  Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_endeff_frame_status()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------
    mavlink_endeff_frame_status_t endeff_frame = endeff_frame_status;
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
    mavlink_msg_endeff_frame_status_encode(system_id, companion_id, &message, &endeff_frame);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}

// ------------------------------------------------------------------------------
//   Write manipualtor joint status Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_joint_status()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_manipulator_joint_status_t jonit_status = mani_joints;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_manipulator_joint_status_encode(system_id, companion_id, &message, &jonit_status);


    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

    return;
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

    printf("\n Start Autopilot_interface read_thread \n");

    result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
    if ( result ) throw result;

//	// now we're reading messages
    printf("\n");

	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
    printf("\n Start Autopilot_interface write_thread \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------

//    printf("CHECK FOR MESSAGES\n");

//    while ( not system_id )
//    {
//        if ( time_to_exit )
//            return;
//        usleep(500000); // check at 2Hz
//    }

//    printf("Found\n");

//    // now we know autopilot is sending messages
//    printf("\n");


//	// wait for it to be started
//	while ( not writing_status )
//        usleep(100000); // 10Hz
//	printf("\n");
//	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
    printf("Close Autopilot_Interface threads \n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{
    printf("start_read_thread\n");

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{

	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{
	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;
    printf("read_thread\n");

	while ( ! time_to_exit )
	{
		read_messages();
        usleep(1000); // Read batches at 100Hz
	}

	reading_status = false;

	return;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
//	writing_status = 2;

	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{

  //      printf("write_joint_status write: \n joint_posi_2=%f; joint_posi_3=%f;\n",mani_joints.joint_posi_2 ,mani_joints.joint_posi_3);
        usleep(10000);   // Stream at 100Hz
        write_endeff_frame_status();
        write_joint_status();
	}

	// signal end
	writing_status = false;

	return;

}
// End Autopilot_Interface

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



