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

void IEKF::correctDistance(const distance_sensor_s *msg)
{
	// require attitude to be initialized
	if (!_attitudeInitialized) {
		return;
	}

	// get correct sensor
	Sensor *sensor = NULL;

	if (msg->type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) {
		sensor = &_sensorSonar;

	} else if (msg->type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) {
		sensor = &_sensorLidar;

	} else {
		return;
	}

	// return if no new data
	float dt = 0;

	if (!sensor->ready(msg->timestamp, dt)) {
		return;
	}

	// if not pointing down (roll 180 by convention), do not use
	if (msg->orientation != 8) {
		ROS_INFO("distance sensor wrong orientation %d", msg->orientation);
		return;
	}

	// if above max distance/ <= 0, out of range
	if (msg->current_distance > msg->max_distance ||
	    msg->current_distance < msg->min_distance) {
		return;
	}

	// if below 0, don't correct and warn
	if (msg->current_distance < 0) {
		ROS_WARN("distance below 0");
		return;
	}

	//ROS_INFO("correct dist bottom");

	// attitude info
	Dcmf C_nb = Quaternion<float>(
			    _x(X::q_nb_0), _x(X::q_nb_1),
			    _x(X::q_nb_2), _x(X::q_nb_3));

	// abort if too large of an angle
	if (C_nb(2, 2) < 1e-1f) {
		ROS_INFO("dist bototm correction aborted, too large of an angle");
		return;
	}

	// init origin alt
	//if (!_origin.altInitialized()) {
	//ROS_INFO("dist bottom  origin init alt %12.2f m", double(msg->current_distance));
	//_origin.altInitialize(msg->current_distance, msg->timestamp);
	//_x(X::asl) = msg->current_distance;
	//_x(X::terrain_asl) = 0;
	//}

	// expected measurement
	float agl = _x(X::asl) - _x(X::terrain_asl);
	float yh = agl / C_nb(2, 2);

	// measured airspeed
	float y = msg->current_distance;

	Vector<float, 1> r;
	r(0) = y - yh;

	// define R
	SquareMatrix<float, Y_distance_down::n> R;
	R(Y_distance_down::d, Y_distance_down::d) = 1e-4f / dt;

	// define H
	// Note: this measurement is not invariant due to
	// rotation matrix
	Matrix<float, Y_distance_down::n, Xe::n> H;
	float x0 = 2 * agl / C_nb(2, 2) / C_nb(2, 2);
	float x1 = 1 / C_nb(2, 2);
	H(Y_distance_down::d, Xe::rot_N) = -C_nb(1, 2) * x0;
	H(Y_distance_down::d, Xe::rot_E) = C_nb(0, 2) * x0;
	H(Y_distance_down::d, Xe::asl) = x1;
	H(Y_distance_down::d, Xe::terrain_asl) = -x1;

	// kalman correction
	SquareMatrix<float, Y_distance_down::n> S;
	sensor->kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov.hagl_innov = r(0);
	_innov.hagl_innov_var = S(0, 0);

	if (sensor->shouldCorrect()) {
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
