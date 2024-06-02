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
 * @file flifo_control.cpp
 * Implementation of the flifo controller. This module receives data from the 
 * multicopter controllers and processes it. It flips the attitude setpoint 
 * when up-side-down flying is requested and generates an attitude trajectory 
 * for up-side-down <-> right-side-up transitions.
 *
 * @author Marco Ruggia		<marco.ruggia@fhgr.ch>
 *
 */

#include "flifo_control.h"
#include <px4_platform_common/events.h>
#include <uORB/Publication.hpp>

using namespace matrix;
using namespace time_literals;

FlifoControl::FlifoControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{	

	poll_parameters();

	_vehicle_attitude_setpoint_pub.advertise();
	_vehicle_rates_setpoint_pub.advertise();
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

FlifoControl::~FlifoControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


bool FlifoControl::init()
{
	_virtual_thrust_setpoint_sub.registerCallback();
	_virtual_torque_setpoint_sub.registerCallback();

	_flifo_status_pub.publish(_flifo_status);

	return true;
}

int FlifoControl::task_spawn(int argc, char *argv[])
{
	FlifoControl *instance = new FlifoControl();

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

int FlifoControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlifoControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
flifo_control is the flifo attitude controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("flifo_control", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FlifoControl::print_status()
{	
	PX4_INFO_RAW("Running\n");
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

void FlifoControl::Run()
{
	if (should_exit()) {
		_virtual_thrust_setpoint_sub.unregisterCallback();
		_virtual_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
	hrt_abstime now = hrt_absolute_time();

	poll_parameters();
	poll_action_request();

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		update_attitude();
	}

	if ( _vehicle_rates_sub.update(&_vehicle_rates)) {
		update_rates();
	}

	update_flip_setpoint();

	if (_virtual_attitude_setpoint_sub.update(&_virtual_attitude_setpoint)) {
		update_attitude_setpoint();
		_vehicle_attitude_setpoint.timestamp = now;
		_vehicle_attitude_setpoint_pub.publish(_vehicle_attitude_setpoint);
	}

	if (_virtual_rates_setpoint_sub.update(&_virtual_rates_setpoint)) {
		update_rates_setpoint();
		_vehicle_rates_setpoint.timestamp = now;
		_vehicle_rates_setpoint_pub.publish(_vehicle_rates_setpoint);
	}

	if (_virtual_thrust_setpoint_sub.update(&_virtual_thrust_setpoint)) {
		update_thrust_setpoint();
		_vehicle_thrust_setpoint.timestamp = now;
		_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);
	}

	if (_virtual_torque_setpoint_sub.update(&_virtual_torque_setpoint)) {
		update_torque_setpoint();
		_vehicle_torque_setpoint.timestamp = now;
		_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
	}

	perf_end(_loop_perf);
}

int flifo_control_main(int argc, char *argv[])
{
	return FlifoControl::main(argc, argv);
}

//##############################################################################


void FlifoControl::poll_parameters()
{
	parameter_update_s param_update;
	
	if (_parameter_update_sub.update(&param_update)) {
		updateParams();
	}
}

void FlifoControl::poll_action_request()
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

void FlifoControl::_set_status(uint8_t state)
{
	if (state != _flifo_status.state) {
		_last_transition = hrt_absolute_time();
		_last_transition_throttle = -1.0f*(float)fabs(_vehicle_thrust_setpoint.xyz[2]);

		_flifo_status.timestamp = hrt_absolute_time();
		_flifo_status.state = state;
		_flifo_status_pub.publish(_flifo_status);
	}
}
void FlifoControl::_set_inv(bool is_inv)
{
	if (is_inv != _flifo_status.is_inv) {

		static hrt_abstime last = 0;
		hrt_abstime now = hrt_absolute_time();
		if ( (now - last) / 1e6f < 0.5f ) { return; }
		last = now;

		_flifo_status.is_inv = is_inv;
		_flifo_status.timestamp = now;
		_flifo_status_pub.publish(_flifo_status);
	}
}


//##############################################################################

void FlifoControl::update_attitude()
{	
	// calculate if flifo is inverted
	Quatf q = Quatf(_vehicle_attitude.q);
	Vector3f up = q.rotateVector(Vector3f(0.0f, 0.0f, 1.0f));
	bool is_inv = ( (up(2)>0.0f) ? false : true );
	_set_inv(is_inv);

	// check if inverted status is compatible with current flifo state
	if ( (_flifo_status.is_inv == true  && _flifo_status.state == flifo_status_s::FLIFO_STATE_RSU) ||
		 (_flifo_status.is_inv == false && _flifo_status.state == flifo_status_s::FLIFO_STATE_USD) ) {
		if (_is_attitude_valid == true) { 
			_is_attitude_valid = false;
			PX4_ERR("[flifo_control] invalid attitude for current flifo state!");
		}
	} else {
		if (_is_attitude_valid == false) {
			_is_attitude_valid = true;
			PX4_WARN("[flifo_control] restored valid attitude for current flifo state!");
		}
	}
}

void FlifoControl::update_rates() {
	// pass
}

void FlifoControl::update_flip_setpoint()
{
	// skip if not currently flipping
	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD || _flifo_status.state == flifo_status_s::FLIFO_STATE_RSU) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_NOFLIP;
		return;
	}

	// calculate key points in flip trajectory 
	static float T, Xacc, Xdec;
	static float t1, t2, t3, t4;
	static float Vmax, Aacc, Adec;
	static float p1, p2, p3, p4;
	if (_flifo_flip.phase == _flifo_flip_s::FLIFO_NOFLIP) {
		T = _param_flifo_rot_tme.get();
		Xacc = _param_flifo_rot_x_acc.get();
		Xdec = _param_flifo_rot_x_dec.get();
		t1 = _param_flifo_spk_tme.get();
		t2 = t1 + Xacc*T;
		t3 = t2 + (1.0f-Xacc-Xdec)*T;
		t4 = t3 + Xdec*T;
		Vmax = M_PI_F / T / ( Xacc/2.0f + Xdec/2.0f + (1.0f-Xacc-Xdec) );
		Aacc = Vmax / (t2-t1);
		Adec = Vmax / (t4-t3);
		p1 = 0.0f;
		p2 = p1 + Vmax/2.0f * (t2-t1);
		p3 = p2 + Vmax      * (t3-t2);
		p4 = p3 + Vmax/2.0f * (t4-t3);
	}

	// calculate current pitch state
	float t = (hrt_absolute_time() - _last_transition) / 1e6f;
	Quatf q = Quatf(_vehicle_attitude.q);
	Eulerf e = Eulerf(q);
	float p = e(1);
	float dp = _vehicle_rates.xyz[1];
	Vector3f up = q.rotateVector(Vector3f(0.0f, 0.0f, 1.0f));
	if (up(2)<0.0f) {
		p = M_PI_F - p;
	}
	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
		p = M_PI_F - p;
		dp = -dp;
	}

	// calculate pitch angle/velocity setpoint in flip trajectory (based on time)
	if (t < t1) {
		_flifo_flip.ang = 0.0f;
		_flifo_flip.vel = 0.0f;
	} else if (t < t2) {
		_flifo_flip.ang = p1 + 0.5f*Aacc*(t-t1)*(t-t1);
		_flifo_flip.vel = Aacc*(t-t1);
	} else if (t < t3) {
		_flifo_flip.ang = p2 + Vmax*(t-t2);
		_flifo_flip.vel = Vmax;
	} else if (t < t4) {
		_flifo_flip.ang = p3 + Vmax*(t-t3) - 0.5f*Adec*(t-t3)*(t-t3);
		_flifo_flip.vel = Vmax - Adec*(t-t3);
	} else {
		_flifo_flip.ang = p4;
		_flifo_flip.vel = 0.0f;
	}

	// calculate torque/thrust setpoint in flip trajectory (based on angle)
	if (t < t1) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_SPIKE;
		if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
			_flifo_flip.frc = _param_flifo_spk_thr1.get() * _last_transition_throttle;
		} else { // (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU)
			_flifo_flip.frc = _param_flifo_spk_thr2.get() * _last_transition_throttle;
		}
		_flifo_flip.trq = 0.0f;

	} else if (p < p2) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_ACCEL;
		_flifo_flip.frc = _param_flifo_rot_thr.get() * _last_transition_throttle;
		if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
			_flifo_flip.trq = _param_flifo_rot_t_acc1.get();
		} else { // (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU)
			_flifo_flip.trq = _param_flifo_rot_t_acc2.get();
		}
		
	} else if (p < p3) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_FREE;
		_flifo_flip.frc = 0.0f;
		_flifo_flip.trq = 0.0f;

	} else if (dp > 0.0f) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_DECEL;
		_flifo_flip.frc = _param_flifo_rot_thr.get() * _last_transition_throttle;
		if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
			_flifo_flip.trq = -_param_flifo_rot_t_dec1.get();
		} else { // (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU)
			_flifo_flip.trq = -_param_flifo_rot_t_dec2.get();
		}

	} else {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_NOFLIP;
		_flifo_flip.frc = 0.0f;
		_flifo_flip.trq = 0.0f;
	}

	// stop trajectory if something went wrong
	if (t > 2.0f*t4) {
		_flifo_flip.phase = _flifo_flip_s::FLIFO_NOFLIP;
		_flifo_flip.frc = 0.0f;
		_flifo_flip.trq = 0.0f;
	}

	// flip trajectory depending on USD->RSU or RSU->USD
	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
		_flifo_flip.ang = M_PI_F - _flifo_flip.ang;
		_flifo_flip.vel = - _flifo_flip.vel;
		_flifo_flip.trq = - _flifo_flip.trq;
	}

	// terminate flip if conditions are met
	if (_flifo_flip.phase == _flifo_flip_s::FLIFO_NOFLIP) {
		if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU_TO_USD) {
			_set_status(flifo_status_s::FLIFO_STATE_USD);
		}
		if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD_TO_RSU) {
			_set_status(flifo_status_s::FLIFO_STATE_RSU);
		}
	}

}

void FlifoControl::update_attitude_setpoint()
{
	// convert attitude setpoint quaternion to euler angles
	Quatf q_sp = Quatf(_virtual_attitude_setpoint.q_d);
	Eulerf euler_sp = Eulerf(q_sp);
	float roll_sp  = euler_sp(0);
	float pitch_sp = euler_sp(1);
	float yaw_sp   = euler_sp(2);

	// modify attitude setpoint based on state
	if (_flifo_status.state == flifo_status_s::FLIFO_STATE_RSU) {
		pitch_sp = pitch_sp;
	} else if (_flifo_status.state == flifo_status_s::FLIFO_STATE_USD) {
		pitch_sp = pitch_sp + M_PI_F;
	} else { // FLIFO_STATE_RSU_TO_USD or FLIFO_STATE_USD_TO_RSU
		pitch_sp = _flifo_flip.ang;
		roll_sp = 0.0f;
	}

	// modify attitude setpoint based on attitude
	if (!_flifo_status.is_inv) {
		roll_sp = roll_sp;
		yaw_sp  = yaw_sp;
	} else {
		roll_sp = -roll_sp;
		yaw_sp  = yaw_sp + M_PI_F;
	}

	// finalize attitude setpoint
	_vehicle_attitude_setpoint = _virtual_attitude_setpoint;
	q_sp = Quatf(Eulerf(roll_sp, pitch_sp, yaw_sp));
	q_sp.copyTo(_vehicle_attitude_setpoint.q_d);
	euler_sp = Eulerf(q_sp);
	_vehicle_attitude_setpoint.roll_body = euler_sp(0);
	_vehicle_attitude_setpoint.pitch_body = euler_sp(1);
	_vehicle_attitude_setpoint.yaw_body = euler_sp(2);

}

void FlifoControl::update_rates_setpoint()
{
	// suppress spikes due to heading flip
	static int spiked_rates = 0;
	static Vector3f prev_rates = Vector3f(_virtual_rates_setpoint.roll, _virtual_rates_setpoint.pitch, _virtual_rates_setpoint.yaw);
	Vector3f curr_rates	= Vector3f(_virtual_rates_setpoint.roll, _virtual_rates_setpoint.pitch, _virtual_rates_setpoint.yaw);
	float diff_rates = (prev_rates-curr_rates).norm_squared();
	float lim_diff_rates = (300.0f*300.0f)*(M_PI_F/180.0f*M_PI_F/180.0f);
	if ( diff_rates > lim_diff_rates && spiked_rates < 8 ) {
		_vehicle_rates_setpoint.timestamp = _virtual_rates_setpoint.timestamp;
		spiked_rates += 1;
		return;
	}
	spiked_rates = 0;
	prev_rates   = curr_rates;

	// use virtual rates setpoint
	_vehicle_rates_setpoint = _virtual_rates_setpoint;

	// add feedforward rate setpoint during transition
	if (_flifo_flip.phase >= _flifo_flip_s::FLIFO_ACCEL && _flifo_flip.phase <= _flifo_flip_s::FLIFO_DECEL) {
		_vehicle_rates_setpoint.pitch += _flifo_flip.vel;
	}
}

void FlifoControl::update_torque_setpoint()
{
	// use virtual torque setpoint
	_vehicle_torque_setpoint = _virtual_torque_setpoint;

	// scale rate control K gain if up-side-down
	if (_flifo_status.is_inv) {
		_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[0] / _param_mc_rollrate_k.get()  * _param_flifo_usd_rr_k.get();
		_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] / _param_mc_pitchrate_k.get() * _param_flifo_usd_pr_k.get();
		_vehicle_torque_setpoint.xyz[2] = _vehicle_torque_setpoint.xyz[2] / _param_mc_yawrate_k.get()   * _param_flifo_usd_yr_k.get();
	}

	// add feedforward torque setpoint during transition
	if (_flifo_flip.phase == _flifo_flip_s::FLIFO_ACCEL || _flifo_flip.phase == _flifo_flip_s::FLIFO_DECEL) {
		_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[0] * _param_flifo_rot_rr_k.get();
		_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.xyz[1] * _param_flifo_rot_pr_k.get() + _flifo_flip.trq;
		_vehicle_torque_setpoint.xyz[2] = _vehicle_torque_setpoint.xyz[2] * _param_flifo_rot_yr_k.get();
	} else if (_flifo_flip.phase == _flifo_flip_s::FLIFO_FREE) {
		_vehicle_torque_setpoint.xyz[0] = 0.0f;
		_vehicle_torque_setpoint.xyz[1] = 0.0f;
		_vehicle_torque_setpoint.xyz[2] = 0.0f;
	}

	// zero torque setpoint if attitude is invalid
	if (!_is_attitude_valid) {
		_vehicle_torque_setpoint.xyz[0] = 0.0f;
		_vehicle_torque_setpoint.xyz[1] = 0.0f;
		_vehicle_torque_setpoint.xyz[2] = 0.0f;
	}
}


void FlifoControl::update_thrust_setpoint()
{
	// use virtual thrust setpoint
	_vehicle_thrust_setpoint = _virtual_thrust_setpoint;

	// add feedforward thrust setpoint during transition
	if (_flifo_flip.phase >= _flifo_flip_s::FLIFO_SPIKE && _flifo_flip.phase <= _flifo_flip_s::FLIFO_DECEL) {
		_vehicle_thrust_setpoint.xyz[2] = _flifo_flip.frc;
	}

	// flip thrust setpoint if up-side-down
	if (_flifo_status.is_inv) {
		_vehicle_thrust_setpoint.xyz[2] = -_vehicle_thrust_setpoint.xyz[2];
	}

	// zero thrust setpoint if attitude is invalid
	if (!_is_attitude_valid) {
		_vehicle_thrust_setpoint.xyz[2] = 0.0;
	}
}
