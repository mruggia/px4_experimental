/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
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
 * @file flifo_control_params.c
 * Parameters for flifo controller.
 *
 * @author Marco Ruggia <marco.ruggia@fhgr.ch>
 */


/**
 * Yaw rate controller gain (when flifo up-side-down)
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_USD_YR_K, 1.0f);
/**
 * Pitch rate controller gain (when flifo up-side-down)
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_USD_PR_K, 1.0f);
/**
 * Roll rate controller gain (when flifo up-side-down)
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_USD_RR_K, 1.0f);
/**
 * Yaw rate controller gain (when flifo in transition)
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_YR_K, 1.0f);
/**
 * Pitch rate controller gain (when flifo in transition)
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_PR_K, 1.0f);
/**
 * Roll rate controller gain (when flifo in transition)
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_RR_K, 1.0f);

/**
 * Hover throttle correction independent of geometry when up-side-down
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_THR_CORR, 1.0f);
/**
 * Minimum throttle to avoid arms oscillating
 *
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_THR_MIN, 0.0f);

/**
 * Duration of throttle spike before transition [s]
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_SPK_TME, 0.0f);
/**
 * Factor of hover throttle spike before flip RSU->USD
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_SPK_THR1, 0.0f);
/**
 * Factor of hover throttle spike before flip USD->RSU
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_SPK_THR2, 0.0f);


/**
 * Duration of flip transition [s]
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_TME, 0.0f);
/**
 * Factor of hover throttle during transition
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_THR, 0.0f);
/**
 * Percentage of flip transition spent accelerating
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_X_ACC, 0.0f);
/**
 * Percentage of flip transition spent decellerating
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_X_DEC, 0.0f);
/**
 * Feedforward flip torque when accelerating RSU->USD
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_T_ACC1, 0.0f);
/**
 * Feedforward flip torque when accelerating USD->RSU
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_T_ACC2, 0.0f);
/**
 * Feedforward flip torque when decellerating RSU->USD
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_T_DEC1, 0.0f);
/**
 * Feedforward flip torque when decellerating USD->RSU
 * 
 * @group FLIFO Control
 */
PARAM_DEFINE_FLOAT(FLIFO_ROT_T_DEC2, 0.0f);

