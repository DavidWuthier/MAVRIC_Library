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
 * \file stabilisation_birotors.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/

#ifndef STABILISATION_BIROTOR
#define STABILISATION_BIROTOR


#ifdef __cplusplus
extern "C" {
#endif

#include "position_estimation.h"
#include "imu.h"
#include "servos.h"
#include "mavlink_waypoint_handler.h"

#include "attitude_controller_p2.h"



/**
 * \brief							Initialize module stabilization
 *
 * \param	stabilisation_copter	The pointer to the stabilisation copter structure
 * \param	stabiliser_conf			The pointer to structure with all PID controllers
 * \param	control_input			The pointer to the controlling inputs
 * \param	imu						The pointer to the IMU structure
 * \param	ahrs		The pointer to the attitude estimation structure
 * \param	pos_est					The pointer to the position estimation structure
 * \param	servos					The pointer to the array of servos command values
 * \param	mavlink_stream			The pointer to the mavlink stream
 */
void stabilisation_birotor_init(attitude_controller_p2_t* stabilisation_birotor, stabilise_birotor_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimator_t* pos_est,servos_t* servos/*, const mavlink_stream_t* mavlink_stream*/);

/**
 * \brief							Main Controller for controlling and stabilizing the quad in position (not using velocity control)
 *
 * \param	stabilisation_copter	The stabilisation structure
 * \param	input					The control command structure
 * \param	waypoint_handler		The waypoint handler structure, to get hold_position coordinates
 * \param	position_estimator		The position estimator structure to compute position error
 */
void stabilisation_birotor_position_hold(attitude_controller_p2_t* stabilisation_birotor, const control_command_t* input, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator);

/**
 * \brief							Main Controller for controlling and stabilizing the quad
 *
 * \param	stabilisation_copter	The stabilisation structure
 */
void stabilisation_birotor_cascade_stabilise(attitude_controller_p2_t* stabilisation_birotor , command_t* command );



#ifdef __cplusplus
}
#endif

#endif
