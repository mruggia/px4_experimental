/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file flifo_att_control.cpp
 * Implementation of the flifo attitude controller. This module receives data from the multicopter
 * attitude controller and processes it. It flips the attitude setpoint when up-side-down flying is
 * requested and generates an attitude trajectory for up-side-down <-> right-side-up transitions.
 *
 * @author Marco Ruggia		<marco.ruggia@fhgr.ch>
 *
 */

#include "flifo_att_control.h"
#include <px4_platform_common/events.h>
#include <uORB/Publication.hpp>

using namespace matrix;
using namespace time_literals;

FlifoAttitudeControl::FlifoAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{	

	poll_parameters();

	_vehicle_attitude_setpoint_pub.advertise();
	_vehicle_thrust_setpoint_pub.advertise();
	_vehicle_torque_setpoint_pub.advertise();

	_flifo_status.timestamp = hrt_absolute_time();
	_flifo_status.state = flifo_status_s::FLIFO_STATE_RSU;
	_flifo_status.is_inv = false;
	_flifo_status_pub.advertise();

	_last_transition = 0;
	_last_transition_throttle = 0.0;
	_is_attitude_valid = true;
}

FlifoAttitudeControl::~FlifoAttitudeControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


bool FlifoAttitudeControl::init()
{
	_virtual_thrust_setpoint_sub.registerCallback();
	_virtual_torque_setpoint_sub.registerCallback();

	_flifo_status_pub.publish(_flifo_status);

	return true;
}

int FlifoAttitudeControl::task_spawn(int argc, char *argv[])
{
	FlifoAttitudeControl *instance = new FlifoAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FlifoAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlifoAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
flifo_att_control is the flifo attitude controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("flifo_att_control", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FlifoAttitudeControl::print_status()
{	
	PX4_INFO_RAW("Running\n");
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

void FlifoAttitudeControl::Run()
{
	if (should_exit()) {
		_virtual_thrust_setpoint_sub.unregisterCallback();
		_virtual_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	poll_parameters();
	poll_action_request();
	poll_vehicle_cmd();

	float dt = (hrt_absolute_time() - _last_transition) / 1e6f;
	if (dt < _param_flifo_spk_tme.get()) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_SPIKE;
		_flifo_flip.progress = dt/_param_flifo_spk_tme.get();
	} else if (dt < _param_flifo_spk_tme.get() + _param_flifo_rot_tme.get()*0.5f) {
		dt = dt - _param_flifo_spk_tme.get();
		_flifo_flip.phase = _flifo_flip_s::FLIFO_PITCH_UP;
		_flifo_flip.progress = dt/(_param_flifo_rot_tme.get()*0.5f);
	} else if (dt < _param_flifo_spk_tme.get() + _param_flifo_rot_tme.get()*1.0f) {
		dt = dt - _param_flifo_spk_tme.get() - _param_flifo_rot_tme.get()*0.5f;
		_flifo_flip.phase = _flifo_flip_s::FLIFO_PITCH_DOWN;
		_flifo_flip.progress = dt/(_param_flifo_rot_tme.get()*0.5f);
	} else if (dt < _param_flifo_spk_tme.get() + _param_flifo_rot_tme.get() + _param_flifo_stb_tme.get()) { 
		dt = dt - _param_flifo_spk_tme.get() - _param_flifo_rot_tme.get();
		_flifo_flip.phase = _flifo_flip_s::FLIFO_STABILIZE;
		_flifo_flip.progress = dt/_param_flifo_stb_tme.get();
	} else {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_NOFLIP;
		_flifo_flip.progress = 0.0f;
	}

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		update_attitude();
	}

	if (_virtual_attitude_setpoint_sub.update(&_virtual_attitude_setpoint)) {
		update_attitude_setpoint();
		_vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
		_vehicle_attitude_setpoint_pub.publish(_vehicle_attitude_setpoint);
	}

	if (_virtual_thrust_setpoint_sub.update(&_virtual_thrust_setpoint)) {
		update_thrust_setpoint();
		_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
		_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);
	}

	if (_virtual_torque_setpoint_sub.update(&_virtual_torque_setpoint)) {
		update_torque_setpoint();
		_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
		_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
	}

	perf_end(_loop_perf);
}

int flifo_att_control_main(int argc, char *argv[])
{
	return FlifoAttitudeControl::main(argc, argv);
}

//##############################################################################


void FlifoAttitudeControl::poll_parameters()
{
	parameter_update_s param_update;
	
	if (_parameter_update_sub.update(&param_update)) {
		updateParams();
	}
}

void FlifoAttitudeControl::poll_vehicle_cmd()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_cmd_sub.update(&vehicle_command)) {
		// parse commands...
	}
}

void FlifoAttitudeControl::poll_action_request()
{
	action_request_s action_request;

	while (_action_request_sub.update(&action_request)) {

		if (action_request.action == action_request_s::ACTION_FLIFO_USD_TO_RSU) {
			_set_status(flifo_status_s::FLIFO_STATE_USD_TO_RSU);
		} else if (action_request.action == action_request_s::ACTION_FLIFO_RSU_TO_USD) {
			_set_status(flifo_status_s::FLIFO_STATE_RSU_TO_USD);
		}
	}
}

void FlifoAttitudeControl::_set_status(uint8_t state)
{
	if (state != _flifo_status.state) {
		_last_transition = hrt_absolute_time();
		_last_transition_throttle = -1.0f*(float)fabs(_vehicle_thrust_setpoint.xyz[2]);

		_flifo_status.timestamp = hrt_absolute_time();
		_flifo_status.state = state;
		_flifo_status_pub.publish(_flifo_status);
	}
}
void FlifoAttitudeControl::_set_inv(bool is_inv)
{
	if (is_inv != _flifo_status.is_inv) {
		static hrt_abstime last = 0;
		hrt_abstime now = hrt_absolute_time();
		if ( (now - last) / 1e6f < 0.5f ) { return; }
		last = now;

		_flifo_status.timestamp = now;
		_flifo_status.is_inv = is_inv;
		_flifo_status_pub.publish(_flifo_status);
	}
}

//##############################################################################


void FlifoAttitudeControl::update_attitude()
{
	Quatf q = Quatf(_vehicle_attitude.q);
	Vector3f up = q.rotateVector(Vector3f(0.0f, 0.0f, 1.0f));
	bool is_inv = ( (up(2)>0.0f) ? false : true );
	_set_inv(is_inv);

	if ( (_flifo_status.is_inv == true  && _flifo_status.state == flifo_status_s::FLIFO_STATE_RSU) ||
		 (_flifo_status.is_inv == false && _flifo_status.state == flifo_status_s::FLIFO_STATE_USD) ) {
		if (_is_attitude_valid == true) { 
			_is_attitude_valid = false;
			PX4_ERR("[flifo_att_control] invalid attitude for current flifo state!");
		}
	} else {
		if (_is_attitude_valid == false) {
			_is_attitude_valid = true;
			PX4_WARN("[flifo_att_control] restored valid attitude for current flifo state!");
		}
	}
}

void FlifoAttitudeControl::update_attitude_setpoint()
{
	float pitch_flifo = 0.0f;
	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU) {
		pitch_flifo = 0.0f;

	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD) {
		pitch_flifo = M_PI_F;

	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
		if (_flifo_flip.phase == _flifo_flip_s::FLIFO_SPIKE) {
			pitch_flifo = 0.0f;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_UP) {
			pitch_flifo = _flifo_flip.progress * M_PI_2_F;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_DOWN) {
			pitch_flifo = _flifo_flip.progress * M_PI_2_F + M_PI_2_F;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_STABILIZE) {
			pitch_flifo = M_PI_F;
		} else { // (_flifo_flip.phase == _flifo_flip_s::FLIFO_NOFLIP)
			pitch_flifo = M_PI_F;
			_set_status(flifo_status_s::FLIFO_STATE_USD);
		}

	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
		if (_flifo_flip.phase == _flifo_flip_s::FLIFO_SPIKE) {
			pitch_flifo = M_PI_F;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_UP) {
			pitch_flifo = (1.0f-_flifo_flip.progress) * M_PI_2_F + M_PI_2_F;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_DOWN) {
			pitch_flifo = (1.0f-_flifo_flip.progress) * M_PI_2_F;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_STABILIZE) {
			pitch_flifo = 0.0f;
		} else { // (_flifo_flip.phase == _flifo_flip_s::FLIFO_NOFLIP)
			pitch_flifo = 0.0f;
			_set_status(flifo_status_s::FLIFO_STATE_RSU);
		}
	}

	_vehicle_attitude_setpoint = _virtual_attitude_setpoint;
	Quatf q_sp = Quatf(_vehicle_attitude_setpoint.q_d);
	Eulerf euler_sp = Eulerf(q_sp);
	float roll_sp  = euler_sp(0);
	float pitch_sp = euler_sp(1);
	float yaw_sp   = euler_sp(2);

	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD || _flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
		pitch_sp = 0.0;
	}

	if (!_flifo_status.is_inv) {
		roll_sp  = roll_sp;
		pitch_sp = pitch_sp + pitch_flifo;
		yaw_sp   = yaw_sp;
	} else {
		roll_sp  = -roll_sp;
		pitch_sp = pitch_sp + pitch_flifo;
		yaw_sp   = yaw_sp + M_PI_F;
	}

	q_sp = Quatf(Eulerf(roll_sp, pitch_sp, yaw_sp));
	q_sp.copyTo(_vehicle_attitude_setpoint.q_d);
	euler_sp = Eulerf(q_sp);
	_vehicle_attitude_setpoint.roll_body = euler_sp(0);
	_vehicle_attitude_setpoint.pitch_body = euler_sp(1);
	_vehicle_attitude_setpoint.yaw_body = euler_sp(2);

}

void FlifoAttitudeControl::update_thrust_setpoint()
{
	float thrust_sp = 0.0f;
	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU || _flifo_status.state == flifo_status_s::FLIFO_STATE_USD) {
		thrust_sp = _virtual_thrust_setpoint.xyz[2];

	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
		if (_flifo_flip.phase == _flifo_flip_s::FLIFO_SPIKE) {
			thrust_sp = _param_flifo_spk_thr1.get() * _last_transition_throttle;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_UP || _flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_DOWN) {
			thrust_sp = _param_flifo_rot_thr.get() * _last_transition_throttle;
		} else {
			thrust_sp = _virtual_thrust_setpoint.xyz[2];
		}

	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
		if (_flifo_flip.phase == _flifo_flip_s::FLIFO_SPIKE) {
			thrust_sp = _param_flifo_spk_thr2.get() * _last_transition_throttle;
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_UP || _flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_DOWN) {
			thrust_sp = _param_flifo_rot_thr.get() * _last_transition_throttle;
		} else {
			thrust_sp = _virtual_thrust_setpoint.xyz[2];
		}
	}

	if (_flifo_status.is_inv) {
		thrust_sp = -thrust_sp;
	}

	if (!_is_attitude_valid) {
		thrust_sp = 0.0;
	}

	_vehicle_thrust_setpoint = _virtual_thrust_setpoint;
	_vehicle_thrust_setpoint.xyz[2] = thrust_sp;

}

void FlifoAttitudeControl::update_torque_setpoint()
{
	_vehicle_torque_setpoint = _virtual_torque_setpoint;

	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
		if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_UP) {
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_k.get() + _param_flifo_rot_ff1.get();
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_DOWN) {
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_k.get() - _param_flifo_rot_ff1.get();
		} else {
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_k.get();
		}

	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
		if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_UP) {
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_k.get() - _param_flifo_rot_ff2.get();
		} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_PITCH_DOWN) {
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_k.get() + _param_flifo_rot_ff2.get();
		} else {
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_k.get();
		}
	}

	if (_flifo_status.is_inv) {
		_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[0] / _param_mc_rollrate_k.get()  * _param_flifo_rollrate_k.get();
		_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] / _param_mc_pitchrate_k.get() * _param_flifo_pitchrate_k.get();
		_vehicle_torque_setpoint.xyz[2] = _vehicle_torque_setpoint.xyz[2] / _param_mc_yawrate_k.get()   * _param_flifo_yawrate_k.get();
	}

	if (!_is_attitude_valid) {
		_vehicle_torque_setpoint.xyz[0] = 0.0;
		_vehicle_torque_setpoint.xyz[1] = 0.0;
		_vehicle_torque_setpoint.xyz[2] = 0.0;
	}
}