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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/action_request.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/flifo_status.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

extern "C" __EXPORT int flifo_att_control_main(int argc, char *argv[]);

class FlifoAttitudeControl : public ModuleBase<FlifoAttitudeControl>, public ModuleParams, public px4::WorkItem
{
public:

	FlifoAttitudeControl();
	~FlifoAttitudeControl() override;

	bool init();

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:

	void Run() override;

	uORB::Subscription 			_action_request_sub{ORB_ID(action_request)};
	uORB::Subscription 			_vehicle_cmd_sub{ORB_ID(vehicle_command)};
	uORB::SubscriptionInterval 	_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription 					_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription 					_vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription 					_virtual_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint_virtual_mc)};
	uORB::Subscription 					_virtual_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint_virtual_mc)};
	uORB::SubscriptionCallbackWorkItem 	_virtual_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint_virtual_mc)};
	uORB::SubscriptionCallbackWorkItem  _virtual_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint_virtual_mc)};

	uORB::Publication<vehicle_attitude_setpoint_s> 	_vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s> 	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<flifo_status_s>				_flifo_status_pub{ORB_ID(flifo_status)};

	vehicle_attitude_s				_vehicle_attitude{};
	vehicle_angular_velocity_s 		_vehicle_rates{};

	vehicle_attitude_setpoint_s 	_virtual_attitude_setpoint{};
	vehicle_rates_setpoint_s		_virtual_rates_setpoint{};
	vehicle_thrust_setpoint_s		_virtual_thrust_setpoint{};
	vehicle_torque_setpoint_s		_virtual_torque_setpoint{};

	vehicle_attitude_setpoint_s		_vehicle_attitude_setpoint{};
	vehicle_rates_setpoint_s		_vehicle_rates_setpoint{};
	vehicle_thrust_setpoint_s		_vehicle_thrust_setpoint{};
	vehicle_torque_setpoint_s		_vehicle_torque_setpoint{};
	flifo_status_s 					_flifo_status{};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	hrt_abstime _last_transition;
	float _last_transition_throttle;
	bool _is_attitude_valid;

	struct _flifo_flip_s {
		enum phases { FLIFO_SPIKE, FLIFO_ACCEL, FLIFO_FREE, FLIFO_DECEL, FLIFO_NOFLIP};
		phases phase = FLIFO_NOFLIP;
		float ang = 0.0f;
		float vel = 0.0f;
		float trq = 0.0f;
		float frc = 0.0f;
	} _flifo_flip;

	void poll_parameters();
	void poll_vehicle_cmd();
	void poll_action_request();

	void update_attitude();
	void update_rates();
	void update_flip_setpoint();
	void update_attitude_setpoint();
	void update_rates_setpoint();
	void update_thrust_setpoint();
	void update_torque_setpoint();

	void _set_status(uint8_t state);
	void _set_inv(bool is_inv);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_YAWRATE_K>)     _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_PITCHRATE_K>)   _param_mc_pitchrate_k,
		(ParamFloat<px4::params::MC_ROLLRATE_K>)    _param_mc_rollrate_k,
		(ParamFloat<px4::params::FLIFO_USD_YR_K>)   _param_flifo_usd_yr_k,
		(ParamFloat<px4::params::FLIFO_USD_PR_K>)   _param_flifo_usd_pr_k,
		(ParamFloat<px4::params::FLIFO_USD_RR_K>)   _param_flifo_usd_rr_k,
		(ParamFloat<px4::params::FLIFO_ROT_YR_K>)   _param_flifo_rot_yr_k,
		(ParamFloat<px4::params::FLIFO_ROT_PR_K>)   _param_flifo_rot_pr_k,
		(ParamFloat<px4::params::FLIFO_ROT_RR_K>)   _param_flifo_rot_rr_k,

		(ParamFloat<px4::params::FLIFO_SPK_TME>)	_param_flifo_spk_tme,
		(ParamFloat<px4::params::FLIFO_SPK_THR1>)	_param_flifo_spk_thr1,
		(ParamFloat<px4::params::FLIFO_SPK_THR2>)	_param_flifo_spk_thr2,

		(ParamFloat<px4::params::FLIFO_ROT_TME>)	_param_flifo_rot_tme,
		(ParamFloat<px4::params::FLIFO_ROT_THR>)	_param_flifo_rot_thr,
		(ParamFloat<px4::params::FLIFO_ROT_X_ACC>)	_param_flifo_rot_x_acc,
		(ParamFloat<px4::params::FLIFO_ROT_X_DEC>)	_param_flifo_rot_x_dec,
		(ParamFloat<px4::params::FLIFO_ROT_T_ACC1>)	_param_flifo_rot_t_acc1,
		(ParamFloat<px4::params::FLIFO_ROT_T_ACC2>)	_param_flifo_rot_t_acc2,
		(ParamFloat<px4::params::FLIFO_ROT_T_DEC1>)	_param_flifo_rot_t_dec1,
		(ParamFloat<px4::params::FLIFO_ROT_T_DEC2>)	_param_flifo_rot_t_dec2
	)

};
