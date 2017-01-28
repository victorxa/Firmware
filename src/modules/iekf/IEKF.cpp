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

#include "IEKF.hpp"
#include "constants.hpp"

float condMaxDefault = 1e3;
float betaMaxDefault = 1e21;

IEKF::IEKF() :
	_nh(), // node handlke
	// sensors
	_sensorAccel("accel", betaMaxDefault, condMaxDefault, 50),
	_sensorMag("mag", betaMaxDefault, condMaxDefault, 50),
	_sensorBaro("baro", betaMaxDefault, condMaxDefault, 50),
	_sensorGps("gps", betaMaxDefault, condMaxDefault, 10),
	_sensorAirspeed("airspeed", betaMaxDefault, condMaxDefault, 0),
	_sensorFlow("flow", betaMaxDefault, condMaxDefault, 10),
	_sensorSonar("sonar", betaMaxDefault, condMaxDefault, 10),
	_sensorLidar("lidar", betaMaxDefault, condMaxDefault, 10),
	_sensorVision("vision", betaMaxDefault, condMaxDefault, 10),
	_sensorMocap("mocap", betaMaxDefault, condMaxDefault, 10),
	_sensorLand("land_detected", betaMaxDefault, condMaxDefault, 0),
	// subscriptions
	_subImu(_nh.subscribe("sensor_combined", 0, &IEKF::callbackImu, this, 1)),
	_subGps(_nh.subscribe("vehicle_gps_position", 0, &IEKF::correctGps, this, 100)),
	_subAirspeed(_nh.subscribe("airspeed", 0, &IEKF::correctAirspeed, this, 100)),
	_subFlow(_nh.subscribe("optical_flow", 0, &IEKF::correctFlow, this, 100)),
	_subDistance(_nh.subscribe("distance_sensor", 0, &IEKF::correctDistance, this, 100)),
	_subVision(_nh.subscribe("vision_position_estimate", 0, &IEKF::correctVision, this, 100)),
	_subMocap(_nh.subscribe("att_pos_mocap", 0, &IEKF::correctMocap, this, 100)),
	_subLand(_nh.subscribe("vehicle_land_detected", 0, &IEKF::callbackLand, this, 100)),
	// publications
	_pubAttitude(_nh.advertise<vehicle_attitude_s>("vehicle_attitude", 0)),
	_pubLocalPosition(_nh.advertise<vehicle_local_position_s>("vehicle_local_position", 0)),
	_pubGlobalPosition(_nh.advertise<vehicle_global_position_s>("vehicle_global_position", 0)),
	_pubControlState(_nh.advertise<control_state_s>("control_state", 0)),
	_pubEstimatorStatus(_nh.advertise<estimator_status_s>("estimator_status", 0)),
	_pubInnov(_nh.advertise<estimator_status_s>("ekf2_innovations", 0)),
	// data
	_x0(),
	_xMin(),
	_xMax(),
	_P0Diag(),
	_x(),
	_P(),
	_u(),
	_g_n(0, 0, -g),
	_origin(),
	_baroAsl(0),
	_baroOffset(0),
	_landed(true),
	_freefall(false),
	_gpsUSec(0),
	_magDeclDeg(0),
	_attitudeInitialized(false),
	_stateTimestamp(),
	_covarianceTimestamp(),
	_imuLowRateIndex(0),
	_accelSaturated(false),
	_gyroSaturated(false),
	_A(), _Q(), _dxe(), _dP(),
	_innov()
{
	// for quaterinons we bound at 2
	// so it has a chance to
	// do a renormalization first
	_xMin(X::q_nb_0) = -1.1;
	_xMin(X::q_nb_1) = -1.1;
	_xMin(X::q_nb_2) = -1.1;
	_xMin(X::q_nb_3) = -1.1;
	_xMin(X::vel_N) = -100;
	_xMin(X::vel_E) = -100;
	_xMin(X::vel_D) = -100;
	_xMin(X::gyro_bias_bX) = -0.1;
	_xMin(X::gyro_bias_bY) = -0.1;
	_xMin(X::gyro_bias_bZ) = -0.1;
	_xMin(X::accel_bias_bX) = -1.0;
	_xMin(X::accel_bias_bY) = -1.0;
	_xMin(X::accel_bias_bZ) = -1.0;
	_xMin(X::pos_N) = -1e30;
	_xMin(X::pos_E) = -1e30;
	_xMin(X::asl) = -1e30;
	_xMin(X::terrain_asl) = -1e30;
	_xMin(X::baro_bias) = -1e30;
	//_xMin(X::wind_N) = -100;
	//_xMin(X::wind_E) = -100;
	//_xMin(X::wind_D) = -100;

	_xMax(X::q_nb_0) = 1.1;
	_xMax(X::q_nb_1) = 1.1;
	_xMax(X::q_nb_2) = 1.1;
	_xMax(X::q_nb_3) = 1.1;
	_xMax(X::vel_N) = 100;
	_xMax(X::vel_E) = 100;
	_xMax(X::vel_D) = 100;
	_xMax(X::gyro_bias_bX) = 0.1;
	_xMax(X::gyro_bias_bY) = 0.1;
	_xMax(X::gyro_bias_bZ) = 0.1;
	_xMax(X::accel_bias_bX) = 1.0;
	_xMax(X::accel_bias_bY) = 1.0;
	_xMax(X::accel_bias_bZ) = 1.0;
	_xMax(X::pos_N) = 1e30;
	_xMax(X::pos_E) = 1e30;
	_xMax(X::asl) = 1e30;
	_xMax(X::terrain_asl) = 1e30;
	_xMax(X::baro_bias) = 1e30;
	//_xMax(X::wind_N) = 100;
	//_xMax(X::wind_E) = 100;
	//_xMax(X::wind_D) = 100;

	// initialize state
	_x0(X::q_nb_0) = 1;
	_x0(X::q_nb_1) = 0;
	_x0(X::q_nb_2) = 0;
	_x0(X::q_nb_3) = 0;
	setX(_x0);

	// initialize covariance
	_P0Diag(Xe::rot_N) = 2;
	_P0Diag(Xe::rot_E) = 2;
	_P0Diag(Xe::rot_D) = 2;
	_P0Diag(Xe::vel_N) = 2;
	_P0Diag(Xe::vel_E) = 2;
	_P0Diag(Xe::vel_D) = 2;
	_P0Diag(Xe::gyro_bias_N) = 0;
	_P0Diag(Xe::gyro_bias_E) = 0;
	_P0Diag(Xe::gyro_bias_D) = 0;
	_P0Diag(Xe::accel_bias_N) = 0;
	_P0Diag(Xe::accel_bias_E) = 0;
	_P0Diag(Xe::accel_bias_D) = 0;
	_P0Diag(Xe::pos_N) = 2;
	_P0Diag(Xe::pos_E) = 2;
	_P0Diag(Xe::asl) = 2;
	_P0Diag(Xe::terrain_asl) = 2;
	_P0Diag(Xe::baro_bias) = 2;
	//_P0Diag(Xe::wind_N) = 0;
	//_P0Diag(Xe::wind_E) = 0;
	//_P0Diag(Xe::wind_D) = 0;
	setP(diag(_P0Diag));


	// magnetic field values for SITL
	// TODO load from GPS or params
	_magDeclDeg = magDeclDeg;


	//_nh.setParam("iekf/test1", 1.0f);
	float test1 = 123;
	_nh.getParam("iekf/test1", test1);
	ROS_INFO("IEKF_TEST1 %10.4f", double(test1));

	float test2 = 456;
	_nh.getParam("iekf/test2", test2);
	ROS_INFO("IEKF_TEST2 %10.4f", double(test2));

	float test3 = 789;
	_nh.getParam("iekf/test3", test3);
	ROS_INFO("IEKF_TEST3 %10.4f", double(test3));
}

Vector<float, X::n> IEKF::dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u)
{
	Vector<float, X::n> dx;
	Quatf q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3f a_b(u(U::accel_bX), u(U::accel_bY), u(U::accel_bZ));
	Vector3f a_bias_b(x(X::accel_bias_bX), x(X::accel_bias_bY), x(X::accel_bias_bZ));
	Vector3f a_b_corrected = a_b - a_bias_b;
	Vector3f a_n = q_nb.conjugate(a_b_corrected);
	Vector3f as_n = a_n - _g_n;
	Vector3f gyro_bias_b(x(X::gyro_bias_bX), x(X::gyro_bias_bY), x(X::gyro_bias_bZ));
	Vector3f omega_nb_b(u(U::omega_nb_bX), u(U::omega_nb_bY), u(U::omega_nb_bZ));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	Quatf dq_nb = q_nb * Quatf(0, omega_nb_b_corrected(0),
				   omega_nb_b_corrected(1), omega_nb_b_corrected(2)) * 0.5f;

	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);

	//ROS_INFO("as_n: %10.4f %10.4f %10.4f",
	//double(as_n(0)),
	//double(as_n(1)),
	//double(as_n(2)));

	//ROS_INFO("V_n: %10.4f %10.4f %10.4f",
	//double(_x(X::vel_N)),
	//double(_x(X::vel_E)),
	//double(_x(X::vel_D)));

	//ROS_INFO("wind: %10.4f %10.4f %10.4f",
	//double(_x(X::wind_N)),
	//double(_x(X::wind_E)),
	//double(_x(X::wind_D)));

	dx(X::gyro_bias_bX) = -_x(X::gyro_bias_bX) / gyro_correlation_time;
	dx(X::gyro_bias_bY) = -_x(X::gyro_bias_bY) / gyro_correlation_time;
	dx(X::gyro_bias_bZ) = -_x(X::gyro_bias_bZ) / gyro_correlation_time;

	dx(X::accel_bias_bX) = -_x(X::accel_bias_bX) / accel_correlation_time;
	dx(X::accel_bias_bY) = -_x(X::accel_bias_bY) / accel_correlation_time;
	dx(X::accel_bias_bZ) = -_x(X::accel_bias_bZ) / accel_correlation_time;

	dx(X::vel_N) = as_n(0);
	dx(X::vel_E) = as_n(1);
	dx(X::vel_D) = as_n(2);

	dx(X::pos_N) = x(X::vel_N);
	dx(X::pos_E) = x(X::vel_E);
	dx(X::asl) = -x(X::vel_D);

	// want terrain dynamics to be static, so when out of range it keeps
	// last estimate and doesn't decay
	dx(X::terrain_asl) = 0;
	dx(X::baro_bias) = -x(X::baro_bias) / baro_correlation_time;
	//dx(X::wind_N) = -_x(X::wind_N) / wind_correlation_time;
	//dx(X::wind_E) = -_x(X::wind_E) / wind_correlation_time;
	//dx(X::wind_D) = -_x(X::wind_D) / wind_correlation_time;
	return dx;
}

void IEKF::callbackImu(const sensor_combined_s *msg)
{
	//ROS_INFO("gyro rate: %f Hz", double(1.0f / dt));

	//ROS_INFO("imu callback");
	_u(U::omega_nb_bX) = msg->gyro_rad[0];
	_u(U::omega_nb_bY) = msg->gyro_rad[1];
	_u(U::omega_nb_bZ) = msg->gyro_rad[2];
	_u(U::accel_bX) = msg->accelerometer_m_s2[0];
	_u(U::accel_bY) = msg->accelerometer_m_s2[1];
	_u(U::accel_bZ) = msg->accelerometer_m_s2[2];


	// update gyro saturation
	if (Vector3f(_u(U::omega_nb_bX),
		     _u(U::omega_nb_bY),
		     _u(U::omega_nb_bZ)).norm() > gyro_saturation_thresh) {
		_gyroSaturated = true;

	} else {
		_gyroSaturated = false;
	}

	// update accel saturation
	if (Vector3f(_u(U::accel_bX),
		     _u(U::accel_bY),
		     _u(U::accel_bZ)).norm() > accel_saturation_thresh) {
		_accelSaturated = true;

	} else {
		_accelSaturated = false;
	}

	if (_attitudeInitialized) {

		// predict driven by gyro callback

		predictState(msg);

		// set correciton deadline to 10 hz
		int lowRateCount = 5;
		uint64_t deadline = msg->timestamp * 1e3 + 1e9 / 10;

		// check if sensors are ready using row late cycle
		if (_imuLowRateIndex % lowRateCount == 0) {
			predictCovariance(msg);

		} else if (_imuLowRateIndex % lowRateCount == 1) {
			correctAccel(msg);

		} else if (_imuLowRateIndex % lowRateCount == 2) {
			correctMag(msg);

		} else if (_imuLowRateIndex % lowRateCount == 3) {
			correctBaro(msg);

		} else if (_imuLowRateIndex % lowRateCount == 4) {
			correctLand(msg->timestamp);
		}

		float overrunMillis = int32_t(ros::Time::now().toNSec() - deadline) / 1.0e6f;

		if (overrunMillis > 1) {
			ROS_WARN("correction deadline exceeded by %10.4f msec",
				 double(overrunMillis));
		}

		_imuLowRateIndex += 1;

	} else {
		initializeAttitude(msg);
	}
}

void IEKF::initializeAttitude(const sensor_combined_s *msg)
{
	// return if no new mag data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (!_sensorMag.ready(timestamp, dt)) {
		return;
	}

	// TODO handle freefall initialization (where accel norm is small and can't be used)
	Vector3f accel = Vector3f(msg->accelerometer_m_s2[0],
				  msg->accelerometer_m_s2[1],
				  msg->accelerometer_m_s2[2]);

	Vector3f mag = Vector3f(
			       msg->magnetometer_ga[0],
			       msg->magnetometer_ga[1],
			       msg->magnetometer_ga[2]).unit();

	Vector3f bz = -accel.unit(); // then only acceleration is likely opposing gravity
	Vector3f bx = (mag - mag.dot(bz) * bz).unit(); // project onto NE plane
	Vector3f by = bz.cross(bx).unit();

	Dcmf C_nb;
	C_nb.setRow(0, bx);
	C_nb.setRow(1, by);
	C_nb.setRow(2, bz);

	// account for magnetic declination
	//Quatf q_nb = Dcmf(Dcmf(AxisAnglef(Vector3f(0, 0, 1), deg2radf*_magDeclDeg))*C_nb);
	Quatf q_nb = C_nb;

	_x(X::q_nb_0) = q_nb(0);
	_x(X::q_nb_1) = q_nb(1);
	_x(X::q_nb_2) = q_nb(2);
	_x(X::q_nb_3) = q_nb(3);

	//for (int i=0; i < 3; i++) {
	//for (int j=0; j < 3; j++) {
	//_P(Xe::gyro_bias_N + i, Xe::gyro_bias_N + j) = 0;
	//if (i == j) {
	//_P(Xe::rot_N + i, Xe::rot_N + j) = 0.1f;
	//} else {
	//_P(Xe::rot_N + i, Xe::rot_N + j) = 0;
	//}
	//}
	//}

	_attitudeInitialized = true;

	Eulerf euler = q_nb;
	ROS_INFO("initial euler angles (deg) roll: %10.4f pitch: %10.4f yaw: %10.4f",
		 double(rad2degf * euler(0)),
		 double(rad2degf * euler(1)),
		 double(rad2degf * euler(2)));
}

void IEKF::predictState(const sensor_combined_s *msg)
{
	// calculate dt
	if (_stateTimestamp == 0) {
		_stateTimestamp = msg->timestamp;
		return;
	}

	float dt = (msg->timestamp - _stateTimestamp) / 1e6f;
	_stateTimestamp = msg->timestamp;

	// make sure dt is reasonable
	if (dt < 0 || dt > 0.1f) {
		return;
	}

	//ROS_INFO("predict state");

	// normalize quaternions if needed
	float qNorm = getQuaternionNB().norm();

	if (fabsf(qNorm - 1.0f) > 1e-3f) {
		ROS_INFO("normalizing quaternion, norm was %6.4f\n",
			 double(qNorm));
		normalizeQuaternion();
	}

	//ROS_INFO("prediction rate period: %10.4g", double(dt));

	Quatf q_nb = getQuaternionNB();

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = dt;
	Vector<float, X::n> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, X::n> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	//ROS_INFO("dx predict \n");
	//dx.print();

	incrementX(dx);
}

void IEKF::predictCovariance(const sensor_combined_s *msg)
{
	// calculate dt
	if (_covarianceTimestamp == 0) {
		_covarianceTimestamp = msg->timestamp;
		return;
	}

	float dt = (msg->timestamp - _covarianceTimestamp) / 1e6f;
	_covarianceTimestamp = msg->timestamp;

	// make sure dt is reasonable
	if (dt < 0 || dt > 0.5f) {
		return;
	}

	//ROS_INFO("predict covariance");

	//ROS_INFO("covariance prediction rate period: %10.4g", double(dt));
	Quatf q_nb = getQuaternionNB();

	// rotation rate
	Vector3f omega_nb_b_corrected = getAngularVelocityNBFrameB();

	// define A matrix
	{
		_A.setZero();

		// derivative of rotation error is -0.5 * gyro bias
		_A(Xe::rot_N, Xe::Xe::gyro_bias_N) = -0.5;
		_A(Xe::rot_E, Xe::Xe::gyro_bias_E) = -0.5;
		_A(Xe::rot_D, Xe::Xe::gyro_bias_D) = -0.5;

		// derivative of velocity
		Vector3f a_b_corrected = getAccelerationFrameB();
		Vector3f J_a_n = q_nb.conjugate(a_b_corrected);
		Matrix<float, 3, 3> a_tmp = -J_a_n.hat() * 2;

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				_A(Xe::vel_N + i, Xe::rot_N + j) = a_tmp(i, j);
			}

			_A(Xe::vel_N + i, Xe::accel_bias_N + i) = -1.0f;
		}

		// derivative of gyro bias
		Vector3f J_omega_n = q_nb.conjugate(omega_nb_b_corrected);
		Matrix<float, 3, 3> g_tmp = J_omega_n.hat();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				_A(Xe::gyro_bias_N + i, Xe::gyro_bias_N + j) = g_tmp(i, j);
			}
		}

		// derivative of position is velocity
		_A(Xe::pos_N, Xe::vel_N) = 1;
		_A(Xe::pos_E, Xe::vel_E) = 1;
		_A(Xe::asl, Xe::vel_D) = -1;

		// derivative of terrain alt is zero

		// derivative of baro bias
		_A(Xe::baro_bias, Xe::baro_bias) = -1.0f / baro_correlation_time;

		// derivative of gyro bias
		_A(Xe::gyro_bias_N, Xe::gyro_bias_N) = -1 / gyro_correlation_time;
		_A(Xe::gyro_bias_E, Xe::gyro_bias_E) = -1 / gyro_correlation_time;
		_A(Xe::gyro_bias_D, Xe::gyro_bias_D) = -1 / gyro_correlation_time;

		// derivative of accel bias
		_A(Xe::accel_bias_N, Xe::accel_bias_N) = -1 / accel_correlation_time;
		_A(Xe::accel_bias_E, Xe::accel_bias_E) = -1 / accel_correlation_time;
		_A(Xe::accel_bias_D, Xe::accel_bias_D) = -1 / accel_correlation_time;

	}

	// define process noise matrix
	{
		float accel_var_rrw = accel_sigma_rrw * accel_sigma_rrw;
		float gyro_var_rrw = gyro_sigma_rrw * gyro_sigma_rrw;
		float rot_var = gyro_sigma_rw * gyro_sigma_rw +  \
				process_noise_sigma_rot * process_noise_sigma_rot;
		float vel_var_xy = accel_sigma_rw * accel_sigma_rw + \
				   process_noise_sigma_vxy * process_noise_sigma_vxy;
		float vel_var_z = accel_sigma_rw * accel_sigma_rw + \
				  process_noise_sigma_vz * process_noise_sigma_vz;
		float baro_var_rrw = baro_sigma_rrw * baro_sigma_rrw;
		float pos_var_xy = process_noise_sigma_xy * process_noise_sigma_xy;
		float pos_var_z = process_noise_sigma_z * process_noise_sigma_z;
		float terrain_var_asl = terrain_sigma_asl * terrain_sigma_asl;

		// account for gyro saturation
		if (getGyroSaturated()) {
			rot_var *= 100;
		}

		// account for accel saturation
		if (getAccelSaturated()) {
			vel_var_z *= 100;
			vel_var_xy *= 100;
		}

		_Q.setZero();
		_Q(Xe::rot_N, Xe::rot_N) = rot_var;
		_Q(Xe::rot_E, Xe::rot_E) = rot_var;
		_Q(Xe::rot_D, Xe::rot_D) = rot_var;
		_Q(Xe::vel_N, Xe::vel_N) = vel_var_xy;
		_Q(Xe::vel_E, Xe::vel_E) = vel_var_xy;
		_Q(Xe::vel_D, Xe::vel_D) = vel_var_z;
		_Q(Xe::gyro_bias_N, Xe::gyro_bias_N) = gyro_var_rrw;
		_Q(Xe::gyro_bias_E, Xe::gyro_bias_E) = gyro_var_rrw;
		_Q(Xe::gyro_bias_D, Xe::gyro_bias_D) = gyro_var_rrw;
		_Q(Xe::accel_bias_N, Xe::accel_bias_N) = accel_var_rrw;
		_Q(Xe::accel_bias_E, Xe::accel_bias_E) = accel_var_rrw;
		_Q(Xe::accel_bias_D, Xe::accel_bias_D) = accel_var_rrw;
		_Q(Xe::pos_N, Xe::pos_N) = pos_var_xy;
		_Q(Xe::pos_E, Xe::pos_E) = pos_var_xy;
		_Q(Xe::asl, Xe::asl) = pos_var_z;
		_Q(Xe::terrain_asl, Xe::terrain_asl) = terrain_var_asl;
		_Q(Xe::baro_bias, Xe::baro_bias) = baro_var_rrw;
		//_Q(Xe::wind_N, Xe::wind_N) = 1e-2f;
		//_Q(Xe::wind_E, Xe::wind_E) = 1e-2f;
		//_Q(Xe::wind_D, Xe::wind_D) = 1e-2f;
	}

	//ROS_INFO("A:");
	//for (int i=0;i<Xe::n; i++) {
	//for (int j=0;j<Xe::n; j++) {
	//printf("%10.3f, ", double(A(i, j)));
	//}
	//printf("\n");
	//}

	// propgate covariance using euler integration
	// add term
	// save stack space by doing this incrementally
	// P+ = P- + (A*P- + P-*A^T + Q)*dt;
	_dP = _A * _P;
	_dP += _P * _A.T();
	_dP += _Q;
	incrementP(_dP * dt);

	//ROS_INFO("P:");
	//_P.diag().print();

	//ROS_INFO("");
	//ROS_INFO("rot (deg) stddev: %10.4f %10.4f %10.4f",
	//double(rad2degf * sqrtf(_P(Xe::rot_N, Xe::rot_N))),
	//double(rad2degf * sqrtf(_P(Xe::rot_E, Xe::rot_E))),
	//double(rad2degf * sqrtf(_P(Xe::rot_D, Xe::rot_D))));
	//ROS_INFO("pos (m) stddev: %10.4f %10.4f %10.4f",
	//double(sqrtf(_P(Xe::pos_N, Xe::pos_N))),
	//double(sqrtf(_P(Xe::pos_E, Xe::pos_E))),
	//double(sqrtf(_P(Xe::asl, Xe::asl))));
	//ROS_INFO("vel (m/s) stddev: %10.4f %10.4f %10.4f",
	//double(sqrtf(_P(Xe::vel_N, Xe::vel_N))),
	//double(sqrtf(_P(Xe::vel_E, Xe::vel_E))),
	//double(sqrtf(_P(Xe::vel_D, Xe::vel_D))));
	//ROS_INFO("accel bias: %10.4f %10.4f %10.4f",
	//double(_x(X::accel_bias_bX)),
	//double(_x(X::accel_bias_bY)),
	//double(_x(X::accel_bias_bZ)));
	//ROS_INFO("accel bias (m/s^2) stddev: %10.4f %10.4f %10.4f",
	//double(sqrtf(_P(Xe::accel_bias_N, Xe::accel_bias_N))),
	//double(sqrtf(_P(Xe::accel_bias_E, Xe::accel_bias_E))),
	//double(sqrtf(_P(Xe::accel_bias_D, Xe::accel_bias_D))));

	//ROS_INFO("baro bias stddev (m) %10.4f", double(sqrtf(_P(Xe::baro_bias, Xe::baro_bias))));
}

Vector<float, X::n> IEKF::computeErrorCorrection(const Vector<float, Xe::n> &d_xe) const
{
	Vector<float, X::n> dx;

	Quatf q_nb = getQuaternionNB();
	Quatf d_q_nb = Quatf(0.0f,
			     d_xe(Xe::rot_N), d_xe(Xe::rot_E), d_xe(Xe::rot_D)) * q_nb;

	//ROS_INFO("d_q_nb");
	//d_q_nb.print();
	Vector3f d_gyro_bias_b = q_nb.conjugate_inversed(
					 Vector3f(d_xe(Xe::gyro_bias_N),
							 d_xe(Xe::gyro_bias_E),
							 d_xe(Xe::gyro_bias_D)));

	Vector3f d_accel_bias_b = q_nb.conjugate_inversed(
					  Vector3f(d_xe(Xe::accel_bias_N),
							  d_xe(Xe::accel_bias_E),
							  d_xe(Xe::accel_bias_D)));


	// linear term correction is the same
	// as the error correction
	dx(X::q_nb_0) += d_q_nb(0);
	dx(X::q_nb_1) += d_q_nb(1);
	dx(X::q_nb_2) += d_q_nb(2);
	dx(X::q_nb_3) += d_q_nb(3);
	dx(X::vel_N) += d_xe(Xe::vel_N);
	dx(X::vel_E) += d_xe(Xe::vel_E);
	dx(X::vel_D) += d_xe(Xe::vel_D);
	dx(X::gyro_bias_bX) += d_gyro_bias_b(0);
	dx(X::gyro_bias_bY) += d_gyro_bias_b(1);
	dx(X::gyro_bias_bZ) +=  d_gyro_bias_b(2);

	dx(X::accel_bias_bX) += d_accel_bias_b(0);
	dx(X::accel_bias_bY) += d_accel_bias_b(1);
	dx(X::accel_bias_bZ) += d_accel_bias_b(2);

	dx(X::pos_N) += d_xe(Xe::pos_N);
	dx(X::pos_E) += d_xe(Xe::pos_E);
	dx(X::asl) += d_xe(Xe::asl);
	dx(X::terrain_asl) += d_xe(Xe::terrain_asl);
	dx(X::baro_bias) += d_xe(Xe::baro_bias);
	//dx(X::wind_N) += d_xe(Xe::wind_N);
	//dx(X::wind_E) += d_xe(Xe::wind_E);
	//dx(X::wind_D) += d_xe(Xe::wind_D);

	correctionLogic(dx);
	return dx;
}


void IEKF::correctionLogic(Vector<float, X::n> &dx) const
{

	//if (!getLanded()) {
	//dx(X::pos_N) = 0;
	//dx(X::pos_E) = 0;
	//}

	//if (!getPositionXYValid()) {
	//dx(X::pos_N) = 0;
	//dx(X::pos_E) = 0;
	//}

	//if (!getAltitudeValid()) {
	//dx(X::asl) = 0;
	//}

	//if (!getVelocityXYValid()) {
	//dx(X::vel_N) = 0;
	//dx(X::vel_E) = 0;
	//}

	//if (!getVelocityZValid()) {
	//dx(X::vel_D) = 0;
	//}

	//if (!getTerrainValid()) {
	//dx(X::terrain_asl) = 0;
	//}
}

void IEKF::boundP()
{
	// force PSD using cholesky decomposition
	// This decomposes the matrix into L * L.T
	// then multiplies it back to zero.
	// The matrix cholesky decomposition nulls
	// any negative components in the decomposition.
	// This isn't strickly the closest PSD matrix
	// to P, but it does the job of keeping the small
	// terms from going negative.
	_P = cholesky(_P);
	_P *= _P.T();

	for (int i = 0; i < Xe::n; i++) {
		// only operate on upper triangle, then copy to lower

		// don't allow NaN or large numbers
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				ROS_WARN("P(%d, %d) NaN, resetting", i, j);

				if (i == j) {
					_P(i, j) = _P0Diag(i);

				} else {
					_P(i, j) = 0;
				}
			}

			if (_P(i, j) > 1e3f) {
				// upper bound
				_P(i, j) = 1e3f;

			} else if (_P(i, j) < -1e3f) {
				// lower bound
				_P(i, j) = -1e3f;
			}
		}

		// force non-negative diagonal
		if (_P(i, i) < 0) {
			ROS_WARN("P(%d, %d) < 0, setting to P0 val", i, i, double(0));
			_P(i, i) = _P0Diag(i);

			for (int k = 0; k < Xe::n; k++) {
				_P(i, k) = 0;
				_P(k, i) = 0;
			}
		}

		// force symmetry, copy uppper triangle to lower
		for (int j = 0; j < i; j++) {
			_P(j, i) = _P(i, j);
		}
	}
}

void IEKF::boundX()
{
	// normalize quaternion
	Quatf q_nb = getQuaternionNB();

	if (fabsf(q_nb.norm() - 1.0f) > 1e-3f) {
		ROS_INFO("normalizing quaternion, norm was %6.4f\n", double(q_nb.norm()));
		normalizeQuaternion();
	}

	// saturate
	for (int i = 0; i < X::n; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			ROS_WARN("x(%d) NaN, setting to %10.4f", i, double(_x0(i)));
			_x(i) = _x0(i);
		}

		if (_x(i) < _xMin(i)) {
			ROS_WARN("x(%d) < lower bound, saturating", i);
			_x(i) = _xMin(i);

		} else if (_x(i) > _xMax(i)) {
			ROS_WARN("x(%d) > upper bound, saturating", i);
			_x(i) = _xMax(i);
		}
	}
}

void IEKF::publish()
{
	if (!_attitudeInitialized) {
		return;
	}

	//ROS_INFO("x:");
	//_x.print();

	//ROS_INFO("P:");
	//_P.diag().print();

	float eph = sqrt(_P(Xe::pos_N, Xe::pos_N) + _P(Xe::pos_E, Xe::pos_E));
	float epv = _P(Xe::asl, Xe::asl);
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Euler<float> euler_nb = q_nb;
	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f a_bias_b(_x(X::accel_bias_bX), _x(X::accel_bias_bY), _x(X::accel_bias_bZ));
	Vector3f a_b_corrected = a_b - a_bias_b;
	Vector3f a_n = q_nb.conjugate(a_b_corrected);
	ros::Time now = ros::Time::now();

	// predicted airspeed
	//Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f wind_n(0, 0, 0);
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float airspeed = -wind_rel_b(0); // body -x component aligned with pitot tube

	// publish attitude
	{
		vehicle_attitude_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.rollspeed = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitchspeed = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yawspeed = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		_pubAttitude.publish(msg);
	}

	// publish local position
	{
		vehicle_local_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.xy_valid = getPositionXYValid();
		msg.z_valid = getAltitudeValid();
		msg.v_xy_valid = getVelocityXYValid();
		msg.v_z_valid = getVelocityZValid();
		msg.x = _x(X::pos_N);
		msg.y = _x(X::pos_E);
		// TODO should make z pub an option
		//msg.z = -getAgl(_x);
		msg.z = -getAltAboveOrigin();
		msg.delta_xy[0] = 0;
		msg.delta_xy[1] = 0;
		msg.delta_z = 0;
		msg.vx = _x(X::vel_N);
		msg.vy = _x(X::vel_E);
		msg.vz = _x(X::vel_D);
		msg.delta_vxy[0] = 0;
		msg.delta_vxy[1] = 0;
		msg.delta_vz = 0;
		msg.xy_reset_counter = 0;
		msg.z_reset_counter = 0;
		msg.vxy_reset_counter = 0;
		msg.vz_reset_counter = 0;
		msg.yaw = euler_nb(2);
		msg.xy_global = _origin.xyInitialized();
		msg.z_global = _origin.altInitialized();
		msg.ref_timestamp = _origin.getXYTimestamp();
		msg.ref_lat = _origin.getLatDeg();
		msg.ref_lon = _origin.getLonDeg();
		msg.ref_alt = _origin.getAlt();
		msg.dist_bottom = getAgl();
		msg.dist_bottom_rate = -_x(X::vel_D);
		msg.surface_bottom_timestamp = now.toNSec() / 1e3;
		msg.dist_bottom_valid = getAglValid();
		msg.eph = eph;
		msg.epv = epv;
		_pubLocalPosition.publish(msg);
	}

	// publish global position
	if (_origin.xyInitialized()
	    && _origin.altInitialized()
	    && getVelocityXYValid()
	    && getVelocityZValid()) {
		double lat_deg = 0;
		double lon_deg = 0;
		_origin.northEastToLatLon(_x(X::pos_N), _x(X::pos_E), lat_deg, lon_deg);
		//ROS_INFO("alt %10.4f m", double(alt_m));
		vehicle_global_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.time_utc_usec = _gpsUSec;
		msg.lat = lat_deg;
		msg.lon = lon_deg;
		msg.alt = _x(X::asl);
		msg.delta_lat_lon[0] = 0;
		msg.delta_lat_lon[1] = 0;
		msg.delta_alt = 0;
		msg.lat_lon_reset_counter = 0;
		msg.alt_reset_counter = 0;
		msg.vel_n = _x(X::vel_N);
		msg.vel_e = _x(X::vel_E);
		msg.vel_d = _x(X::vel_D);
		msg.yaw = euler_nb(2);
		msg.eph = eph;
		msg.epv = epv;
		msg.terrain_alt = _x(X::terrain_asl);
		msg.terrain_alt_valid = getTerrainValid();
		msg.dead_reckoning = false;
		msg.pressure_alt = _baroAsl;
		_pubGlobalPosition.publish(msg);
	}

	// publish control state
	{
		// specific acceleration
		control_state_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.x_acc = a_n(0);
		msg.y_acc = a_n(1);
		msg.z_acc = a_n(2);
		msg.x_vel = _x(X::vel_N);
		msg.y_vel = _x(X::vel_E);
		msg.z_vel = _x(X::vel_D);
		msg.x_pos = _x(X::pos_N);
		msg.y_pos = _x(X::pos_E);
		msg.z_pos = -getAltAboveOrigin();
		msg.airspeed = airspeed;
		msg.airspeed_valid = true;
		msg.vel_variance[0] = _P(Xe::vel_N, Xe::vel_N);
		msg.vel_variance[1] = _P(Xe::vel_E, Xe::vel_E);
		msg.vel_variance[2] = _P(Xe::vel_D, Xe::vel_D);
		msg.pos_variance[0] = _P(Xe::pos_N, Xe::pos_N);
		msg.pos_variance[1] = _P(Xe::pos_E, Xe::pos_E);
		msg.pos_variance[2] = _P(Xe::asl, Xe::asl);
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.delta_q_reset[0] = 0;
		msg.delta_q_reset[1] = 0;
		msg.delta_q_reset[2] = 0;
		msg.delta_q_reset[3] = 0;
		msg.quat_reset_counter = 0;
		msg.roll_rate = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitch_rate = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yaw_rate = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		msg.horz_acc_mag = 0;
		_pubControlState.publish(msg);
	}

	// estimator status
	{
		estimator_status_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.vibe[0] = 0; // TODO
		msg.vibe[1] = 0; // TODO
		msg.vibe[2] = 0; // TODO
		msg.n_states = X::n;

		for (int i = 0; i < X::n; i++) {
			msg.states[i] = _x(i);
		}

		for (int i = 0; i < Xe::n; i++) {
			msg.covariances[i] = _P(i, i);
		}

		// XXX
		// this isn't really general and is
		// tailored to EKF2 so just dumping
		// data in the best field names available
		msg.gps_check_fail_flags = 0; // TODO
		msg.control_mode_flags = 0; // TODO
		msg.filter_fault_flags = 0; // TODO
		msg.pos_horiz_accuracy = eph;
		msg.pos_vert_accuracy = epv;
		msg.innovation_check_flags = 0; // TODO
		msg.mag_test_ratio = _sensorMag.getBeta();
		msg.vel_test_ratio = _sensorFlow.getBeta();
		msg.pos_test_ratio = _sensorGps.getBeta();
		msg.hgt_test_ratio = _sensorAccel.getBeta();
		msg.tas_test_ratio = _sensorAirspeed.getBeta();
		msg.hagl_test_ratio = _sensorLidar.getBeta();
		msg.solution_status_flags = 0; // TODO
		_pubEstimatorStatus.publish(msg);
	}

	// ekf_innovations
	{
		_innov.timestamp = now.toNSec() / 1e3;
		_pubInnov.publish(_innov);
	}
}
