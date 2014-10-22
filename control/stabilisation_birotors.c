/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
 
/*******************************************************************************
 * \file stabilisation_birotors.c *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#include "stabilisation_birotors.h"
#include "print_util.h"

void stabilisation_birotor_init(attitude_controller_p2_t* stabilisation_birotor, stabilise_birotor_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimator_t* pos_est,servos_t* servos/*, const mavlink_stream_t* mavlink_stream*/)
{
	
	stabilisation_birotor->stabiliser_stack = stabiliser_conf->stabiliser_stack;
	stabilisation_birotor->controls = controls;
	stabilisation_birotor->imu = imu;
	stabilisation_birotor->ahrs = ahrs;
	stabilisation_birotor->pos_est = pos_est;
	stabilisation_birotor->servos = servos;
	
	controls->control_mode = ATTITUDE_COMMAND_MODE;
	controls->yaw_mode = YAW_ABSOLUTE;
	
	controls->rpy[ROLL] = 0.0f;
	controls->rpy[PITCH] = 0.0f;
	controls->rpy[YAW] = 0.0f;
	controls->tvel[X] = 0.0f;
	controls->tvel[Y] = 0.0f;
	controls->tvel[Z] = 0.0f;
	controls->theading = 0.0f;
	controls->thrust = -1.0f;

	//stabilisation_birotor->stabiliser_stack.rate_stabiliser.mavlink_stream = mavlink_stream;
	//stabilisation_birotor->stabiliser_stack.attitude_stabiliser.mavlink_stream = mavlink_stream;
	//stabilisation_birotor->stabiliser_stack.velocity_stabiliser.mavlink_stream = mavlink_stream;
	//stabilisation_birotor->stabiliser_stack.position_stabiliser.mavlink_stream = mavlink_stream;
	
	
	print_util_dbg_print("Stabilisation copter init.\r\n");
}

void stabilisation_birotor_position_hold(attitude_controller_p2_t* stabilisation_birotor, const control_command_t* input, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator)
{
	aero_attitude_t attitude_yaw_inverse;
	quat_t q_rot;
	// input = stabilisation_birotor->controls_nav;
	
	attitude_yaw_inverse = coord_conventions_quat_to_aero(stabilisation_birotor->ahrs->qe);
	attitude_yaw_inverse.rpy[0] = 0.0f;
	attitude_yaw_inverse.rpy[1] = 0.0f;
	attitude_yaw_inverse.rpy[2] = -attitude_yaw_inverse.rpy[2];
	
	//qtmp=quaternions_create_from_vector(input.tvel);
	//quat_t input_global = quaternions_local_to_global(stabilisation_birotor->ahrs->qe, qtmp);
	
	q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);
	
	float pos_error[4];
	pos_error[X] = waypoint_handler->waypoint_hold_coordinates.pos[X] - position_estimator->local_position.pos[X];
	pos_error[Y] = waypoint_handler->waypoint_hold_coordinates.pos[Y] - position_estimator->local_position.pos[Y];
	pos_error[3] = -(waypoint_handler->waypoint_hold_coordinates.pos[Z] - position_estimator->local_position.pos[Z]);
	
	pos_error[YAW]= input->rpy[YAW];
	
	// run PID update on all velocity controllers
	stabilisation_run(&stabilisation_birotor->stabiliser_stack.position_stabiliser, stabilisation_birotor->imu->dt, pos_error);
	
	float pid_output_global[3];
	
	pid_output_global[0] = stabilisation_birotor->stabiliser_stack.position_stabiliser.output.rpy[0];
	pid_output_global[1] = stabilisation_birotor->stabiliser_stack.position_stabiliser.output.rpy[1];
	pid_output_global[2] = stabilisation_birotor->stabiliser_stack.position_stabiliser.output.thrust + THRUST_HOVER_POINT;
	
	float pid_output_local[3];
	quaternions_rotate_vector(q_rot, pid_output_global, pid_output_local);
	
	*stabilisation_birotor->controls = *input;
	stabilisation_birotor->controls->rpy[ROLL] = pid_output_local[Y];
	stabilisation_birotor->controls->rpy[PITCH] = -pid_output_local[X];
	stabilisation_birotor->controls->thrust = pid_output_local[2];
	
	stabilisation_birotor->controls->control_mode = ATTITUDE_COMMAND_MODE;//VELOCITY_COMMAND_MODE;
}

void stabilisation_birotor_cascade_stabilise(attitude_controller_p2_t* stabilisation_birotor , command_t* command )
{
	float rpyt_errors[4];
	control_command_t input;
	int32_t i;
	quat_t qtmp, q_rot;
	aero_attitude_t attitude_yaw_inverse;
	
	// set the controller input
	input= *stabilisation_birotor->controls;
	switch (stabilisation_birotor->controls->control_mode) {
	case VELOCITY_COMMAND_MODE:
		
		attitude_yaw_inverse = coord_conventions_quat_to_aero(stabilisation_birotor->ahrs->qe);
		attitude_yaw_inverse.rpy[0] = 0.0f;
		attitude_yaw_inverse.rpy[1] = 0.0f;
		attitude_yaw_inverse.rpy[2] = attitude_yaw_inverse.rpy[2];
		
		//qtmp=quaternions_create_from_vector(input.tvel);
		//quat_t input_global = quaternions_local_to_global(stabilisation_birotor->ahrs->qe, qtmp);
		
		q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);
		
		quat_t input_global;
		// input.tvel comes from the remote if it is in velocity command from remote.
		// But it comes from the navigation in case of waypoint
		quaternions_rotate_vector(q_rot, input.tvel, input_global.v);
		
		input.tvel[X] = input_global.v[X];
		input.tvel[Y] = input_global.v[Y];
		input.tvel[Z] = input_global.v[Z];
		
		rpyt_errors[X] = input.tvel[X] - stabilisation_birotor->pos_est->vel[X];
		rpyt_errors[Y] = input.tvel[Y] - stabilisation_birotor->pos_est->vel[Y];
		rpyt_errors[Z] = -(input.tvel[Z] - stabilisation_birotor->pos_est->vel[Z]);
		
		if (stabilisation_birotor->controls->yaw_mode == YAW_COORDINATED) 
		{
			float rel_heading_coordinated;
			if ((maths_f_abs(stabilisation_birotor->pos_est->vel_bf[X])<0.001f)&&(maths_f_abs(stabilisation_birotor->pos_est->vel_bf[Y])<0.001f))
			{
				rel_heading_coordinated = 0.0f;
			}
			else
			{
				rel_heading_coordinated = atan2(stabilisation_birotor->pos_est->vel_bf[Y], stabilisation_birotor->pos_est->vel_bf[X]);
			}
			
			float w = 0.5f * (maths_sigmoid(vectors_norm(stabilisation_birotor->pos_est->vel_bf)-stabilisation_birotor->stabiliser_stack.yaw_coordination_velocity) + 1.0f);
			input.rpy[YAW] = (1.0f - w) * input.rpy[YAW] + w * rel_heading_coordinated;
		}

		rpyt_errors[YAW]= input.rpy[YAW];
		
		// run PID update on all velocity controllers
		stabilisation_run(&stabilisation_birotor->stabiliser_stack.velocity_stabiliser, stabilisation_birotor->imu->dt, rpyt_errors);
		
		//velocity_stabiliser.output.thrust = maths_f_min(velocity_stabiliser.output.thrust,stabilisation_param.controls->thrust);
		stabilisation_birotor->stabiliser_stack.velocity_stabiliser.output.thrust += THRUST_HOVER_POINT;
		stabilisation_birotor->stabiliser_stack.velocity_stabiliser.output.theading = input.theading;
		input = stabilisation_birotor->stabiliser_stack.velocity_stabiliser.output;
		
		qtmp=quaternions_create_from_vector(stabilisation_birotor->stabiliser_stack.velocity_stabiliser.output.rpy);
		//quat_t rpy_local = quaternions_global_to_local(stabilisation_birotor->ahrs->qe, qtmp);
		
		quat_t rpy_local;
		quaternions_rotate_vector(quaternions_inverse(q_rot), qtmp.v, rpy_local.v);
		
		input.rpy[ROLL] = rpy_local.v[Y];
		input.rpy[PITCH] = -rpy_local.v[X];
		//input.thrust = stabilisation_birotor->controls->tvel[Z];
		
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:

		command->attitude.rpy[ROLL] = input.rpy[ROLL];
		command->attitude.rpy[PITCH] = input.rpy[PITCH];
		command->attitude.rpy[YAW] = input.rpy[YAW]; 

		
		// PART OF THE QUATERNION CONTROLLER
		// Get attitude command
		switch ( stabilisation_birotor->attitude_command->mode )
		{
			case ATTITUDE_COMMAND_MODE_QUATERNION:
			attitude_error_estimator_set_quat_ref(	&stabilisation_birotor->attitude_error_estimator,
			stabilisation_birotor->attitude_command->quat );
			break;

			case ATTITUDE_COMMAND_MODE_RPY:
			attitude_error_estimator_set_quat_ref_from_rpy( &stabilisation_birotor->attitude_error_estimator,
															stabilisation_birotor->attitude_command->rpy );
			break;
		}
		quat_t pitch_offset;
		
		aero_attitude_t aero_offset;
		aero_offset.rpy[0] = stabilisation_birotor->stab_angle[0];
		aero_offset.rpy[1] = stabilisation_birotor->stab_angle[1];
		aero_offset.rpy[2] = 0;
		
		pitch_offset = coord_conventions_quaternion_from_aero(aero_offset);
		
	
	
		quat_t yaw_offset;
				
		aero_attitude_t aero_offset_yaw;
		aero_offset_yaw.rpy[0] = stabilisation_birotor->stab_angle[2];
		aero_offset_yaw.rpy[1] = 0;
		aero_offset_yaw.rpy[2] = 0;

		yaw_offset = coord_conventions_quaternion_from_aero(aero_offset_yaw);
		
		quat_t tot = quaternions_multiply(yaw_offset,pitch_offset);
		
		
		stabilisation_birotor->attitude_error_estimator.quat_ref = quaternions_multiply(stabilisation_birotor->attitude_error_estimator.quat_ref, tot);
		

		// END OF THE PART OF THE QUATERNION CONTROLLER
		attitude_error_estimator_update( &stabilisation_birotor->attitude_error_estimator );
		rpyt_errors[0] = stabilisation_birotor->attitude_error_estimator.rpy_errors[0];
		rpyt_errors[1] = stabilisation_birotor->attitude_error_estimator.rpy_errors[1];
		rpyt_errors[2] = stabilisation_birotor->attitude_error_estimator.rpy_errors[2];
		
		/*rpyt_errors[0] = input.rpy[ROLL];
		rpyt_errors[1] = input.rpy[PITCH];
		rpyt_errors[2] = input.rpy[YAW];*/
		
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude_filter controllers
		stabilisation_run(&stabilisation_birotor->stabiliser_stack.attitude_stabiliser, stabilisation_birotor->imu->dt, rpyt_errors);
		
		// use output of attitude_filter controller to set rate setpoints for rate controller 
		input = stabilisation_birotor->stabiliser_stack.attitude_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++)
		{
			rpyt_errors[i]= input.rpy[i]- stabilisation_birotor->ahrs->angular_speed[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilisation_run(&stabilisation_birotor->stabiliser_stack.rate_stabiliser, stabilisation_birotor->imu->dt, rpyt_errors);
	}
	
	// mix to servo outputs depending on configuration


	command->torque.xyz[0] 	= stabilisation_birotor->stabiliser_stack.rate_stabiliser.output.rpy[0];
	command->torque.xyz[1] 	= stabilisation_birotor->stabiliser_stack.rate_stabiliser.output.rpy[1];
	command->torque.xyz[2] 	= stabilisation_birotor->stabiliser_stack.rate_stabiliser.output.rpy[2];

	command->thrust.thrust  	= stabilisation_birotor->stabiliser_stack.rate_stabiliser.output.thrust;
	
}
