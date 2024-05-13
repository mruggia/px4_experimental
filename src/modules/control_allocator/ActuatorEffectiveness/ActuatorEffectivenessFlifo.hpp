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

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include <uORB/topics/flifo_status.h>

class ActuatorEffectivenessFlifo : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessFlifo(ModuleParams *parent);
	virtual ~ActuatorEffectivenessFlifo() = default;

	const char *name() const override { return "flifo"; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
		{ allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION; }

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override { normalize[0] = false; }

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	/*void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;*/

	flifo_status_s getFlifoStatus() { return _flifo_status; }
	void  setFlifoStatus(flifo_status_s flifo_status) { _flifo_status = flifo_status; }
	float getFlifoUSDThrustFactor() { return _thrust_usd_factor; };
	float getFlifoActuatorMin();
	float getFlifoActuatorMax();

protected:
	ActuatorEffectivenessRotors _mc_rotors;

private:
	flifo_status_s _flifo_status;
	float _thrust_usd_factor;
	
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FLIFO_THR_CORR>)  _param_flifo_thr_corr,
		(ParamFloat<px4::params::FLIFO_THR_MIN>)   _param_flifo_thr_min
	)

};
