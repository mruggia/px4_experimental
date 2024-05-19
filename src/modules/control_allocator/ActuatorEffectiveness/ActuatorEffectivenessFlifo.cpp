/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessFlifo.hpp"

using namespace matrix;

// initialize ActuatorEffectivenessRotors with double the rotors. first half for right-side-up, second half for up-side-down
ActuatorEffectivenessFlifo::ActuatorEffectivenessFlifo(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this)
{	
	_flifo_status.state = flifo_status_s::FLIFO_STATE_RSU;
	_flifo_status.is_inv = false;

}

bool ActuatorEffectivenessFlifo::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_rotors = _mc_rotors.geometry().num_rotors/2;
	int start_index = configuration.num_actuators_matrix[configuration.selected_matrix];

	if (num_rotors*2 + start_index >= NUM_ACTUATORS) {
		PX4_ERR("Actuator count exceeds maximum");
		return false; 
	}

	// calculate effectiveness matrix (actuator 1-4 for right-side-up, actuator 5-8 for up-side down)
	EffectivenessMatrix& effectiveness = configuration.effectiveness_matrices[configuration.selected_matrix];
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(_mc_rotors.geometry(), effectiveness, start_index);

	// get hover thrust factor between right-side-up and up-side-down (thrust is later normalized, so factor will be added back)
	_thrust_usd_factor = _param_flifo_thr_corr.get() * 
		( effectiveness(5,0)+effectiveness(5,1)+effectiveness(5,2)+effectiveness(5,3) ) / 
		( effectiveness(5,4)+effectiveness(5,5)+effectiveness(5,6)+effectiveness(5,7) );

	// extract relevant part of effectiveness matrix based on flifo state
	for (int j = 0; j < num_rotors*2; j++) {

		if (_flifo_status.is_inv) {
			effectiveness(0,j+start_index) = effectiveness(0,j+start_index+num_rotors);
			effectiveness(1,j+start_index) = effectiveness(1,j+start_index+num_rotors);
			effectiveness(2,j+start_index) = effectiveness(2,j+start_index+num_rotors);
		}

		effectiveness(3,j+start_index) = 0.0f;
		effectiveness(4,j+start_index) = 0.0f;
		effectiveness(5,j+start_index) = -1.0f; // has to be normalized!

		if(j >= num_rotors) {
			for (int i = 0; i < NUM_AXES; i++) {
				effectiveness(i,j+start_index) = 0.0f;
			}
		}
	}
	
	// finalize configuration
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_rotors);
	return true;
}

float ActuatorEffectivenessFlifo::getFlifoActuatorMin() 
{	
	float min;
	if (!_flifo_status.is_inv) {
		min = 1.f/1000.f+FLT_EPSILON;		// equal to effective_output = 1001
		min = min + _param_flifo_thr_min.get();
	} else {
		min = -999.f/1000.f-FLT_EPSILON;	// equal to effective_output = 1
	}
	return min;
}
float ActuatorEffectivenessFlifo::getFlifoActuatorMax()
{
	float max;
	if (!_flifo_status.is_inv) {
		max = 999.f/1000.f+FLT_EPSILON;	// equal to effective_output = 1999
	} else {
		max = -1.f/1000.f-FLT_EPSILON;		// equal to effective_output = 999
		max = max - _param_flifo_thr_min.get();
	}
	return max;
}