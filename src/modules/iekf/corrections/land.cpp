/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "../IEKF.hpp"

void IEKF::callbackLand(const vehicle_land_detected_s *msg)
{
	//ROS_INFO("callback land");
	_landed = msg->landed;
	_freefall = msg->freefall;
}

void IEKF::correctLand(uint64_t timestamp)
{
	// return if not landed
	if (!_landed) {
		return;
	}

	// return if no new data or too fast
	float dt = 0;

	if (!_sensorLand.ready(timestamp, dt)) {
		return;
	}

	//ROS_INFO("correct land");

	// calculate residual
	Vector<float, Y_land::n> y;
	y(Y_land::vel_N) = 0;
	y(Y_land::vel_E) = 0;
	y(Y_land::vel_D) = 0;
	y(Y_land::agl) = 0;

	Vector<float, Y_land::n> yh;
	yh(Y_land::vel_N) = _x(X::vel_N);
	yh(Y_land::vel_E) = _x(X::vel_E);
	yh(Y_land::vel_D) = _x(X::vel_D);
	yh(Y_land::agl) = getAgl();

	Vector<float, Y_land::n> r = y - yh;

	// define R
	SquareMatrix<float, Y_land::n> R;
	float var_vxy = land_sigma_vxy * land_sigma_vxy / dt;
	float var_vz = land_sigma_vz * land_sigma_vz / dt;
	float var_agl = land_sigma_agl * land_sigma_agl / dt;

	R(Y_land::vel_N, Y_land::vel_N) = var_vxy;
	R(Y_land::vel_E, Y_land::vel_E) = var_vxy;
	R(Y_land::vel_D, Y_land::vel_D) = var_vz;
	R(Y_land::agl, Y_land::agl) = var_agl;

	// define H
	Matrix<float, Y_land::n, Xe::n> H;
	H(Y_land::vel_N, Xe::vel_N) = 1;
	H(Y_land::vel_E, Xe::vel_E) = 1;
	H(Y_land::vel_D, Xe::vel_D) = 1;
	H(Y_land::agl, Xe::asl) = 1;
	H(Y_land::agl, Xe::terrain_asl) = -1;

	// kalman correction
	SquareMatrix<float, Y_land::n> S;
	_sensorLand.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	// XXX ekf2 doesn't have a place for this innovation

	if (_sensorLand.shouldCorrect()) {
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
