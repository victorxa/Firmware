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

#include "ros/ros.hpp"
#include "matrix/math.hpp"
#include <lib/geo/geo.h>

// local includes
#include "Origin.hpp"
#include "Sensor.hpp"
#include "constants.hpp"

// subscriptions
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/ekf2_innovations.h>

// publications
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/estimator_status.h>

using namespace matrix;

/**
 * Main class for invariant extended kalman filter
 *
 * inspired by: https://hal.archives-ouvertes.fr/hal-00494342/document
 *
 * See: https://github.com/jgoppert/iekf_analysisa for derivation/ simulation
 */
class IEKF
{
public:
	IEKF();
	Vector<float, X::n> dynamics(float t, const Vector<float, X::n> &x,
				     const Vector<float, U::n> &u);
	bool ok() { return _nh.ok(); }
	void callbackImu(const sensor_combined_s *msg);
	void initializeAttitude(const sensor_combined_s *msg);
	void correctAccel(const sensor_combined_s *msg);
	void correctMag(const sensor_combined_s *msg);
	void correctBaro(const sensor_combined_s *msg);
	void correctGps(const vehicle_gps_position_s *msg);
	void correctAirspeed(const airspeed_s *msg);
	void correctFlow(const optical_flow_s *msg);
	void correctDistance(const distance_sensor_s *msg);
	void correctVision(const vision_position_estimate_s *msg);
	void correctMocap(const att_pos_mocap_s *msg);
	void callbackLand(const vehicle_land_detected_s *msg);
	void correctLand(uint64_t timestamp);
	void predictState(const sensor_combined_s *msg);
	void predictCovariance(const sensor_combined_s *msg);
	Vector<float, X::n> computeErrorCorrection(const Vector<float, Xe::n> &d_xe) const;
	void correctionLogic(Vector<float, X::n> &dx) const;
	void boundP();
	void setP(const SquareMatrix<float, Xe::n> &P)
	{
		_P = P;
		boundP();
	}
	inline void incrementP(const SquareMatrix<float, Xe::n> &dP)
	{
		_P += dP;
		boundP();
	}
	void boundX();
	void setX(const Vector<float, X::n> &x)
	{
		_x = x;
		boundX();
	}
	void incrementX(Vector<float, X::n> &dx)
	{
		correctionLogic(dx);
		_x += dx;
		boundX();
	}
	void publish();
	inline bool getLanded() const
	{
		return _landed;
	}
	inline bool getAttitudeValid() const
	{
		return _attitudeInitialized &&
		       ((_P(Xe::rot_N, Xe::rot_N)
			 + _P(Xe::rot_E, Xe::rot_E)
			 + _P(Xe::rot_D, Xe::rot_D)) < 1.0f);

	};
	inline bool getVelocityXYValid() const
	{
		return ((_P(Xe::vel_N, Xe::vel_N)
			 + _P(Xe::vel_E, Xe::vel_E)) < 2.0f);
	};
	inline bool getVelocityZValid() const
	{
		return _P(Xe::vel_D, Xe::vel_D) < 1.0f;
	};

	inline bool getPositionXYValid() const
	{
		return _origin.xyInitialized()
		       && ((_P(Xe::pos_N, Xe::pos_N)
			    + _P(Xe::pos_E, Xe::pos_E)) < 2.0f);
	};
	inline bool getAltitudeValid() const
	{
		return _origin.altInitialized()
		       && (_P(Xe::asl, Xe::asl) < 1.0f);
	};
	inline bool getAglValid() const
	{
		return _origin.altInitialized()
		       && (_P(Xe::asl, Xe::asl) < 1.0f)
		       && (_P(Xe::terrain_asl, Xe::terrain_asl) < 1.0f);
	};
	inline bool getTerrainValid() const
	{
		return (_P(Xe::terrain_asl, Xe::terrain_asl) < 1.0f);
	};
	inline float getAgl() const
	{
		return _x(X::asl) - _x(X::terrain_asl);
	}
	inline float getAltAboveOrigin() const
	{
		return _x(X::asl) - _origin.getAlt();
	};
	inline bool getGyroSaturated() const
	{
		return _gyroSaturated;
	}
	inline bool getAccelSaturated() const
	{
		return _accelSaturated;
	}
	inline Quatf getQuaternionNB() const
	{
		return Quatf(_x(X::q_nb_0), _x(X::q_nb_1),
			     _x(X::q_nb_2), _x(X::q_nb_3));
	}
	inline Dcmf computeDcmNB() const
	{
		return getQuaternionNB();
	}
	inline Vector3f getGyroBiasFrameB() const
	{
		return Vector3f(_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	}
	inline Vector3f getGyroRawFrameB() const
	{
		return Vector3f(_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	}
	inline Vector3f getAngularVelocityNBFrameB() const
	{
		return getGyroRawFrameB() - getGyroBiasFrameB();
	}
	inline Vector3f getAccelerationFrameB() const
	{
		Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
		Vector3f a_bias_b(_x(X::accel_bias_bX), _x(X::accel_bias_bY), _x(X::accel_bias_bZ));
		return a_b - a_bias_b;
	}

	void normalizeQuaternion()
	{
		Quatf q = getQuaternionNB();
		q.normalize();
		_x(X::q_nb_0) = q(0);
		_x(X::q_nb_1) = q(1);
		_x(X::q_nb_2) = q(2);
		_x(X::q_nb_3) = q(3);
	}

private:
	ros::NodeHandle _nh;

	// sensors
	Sensor _sensorAccel;
	Sensor _sensorMag;
	Sensor _sensorBaro;
	Sensor _sensorGps;
	Sensor _sensorAirspeed;
	Sensor _sensorFlow;
	Sensor _sensorSonar;
	Sensor _sensorLidar;
	Sensor _sensorVision;
	Sensor _sensorMocap;
	Sensor _sensorLand;

	// subscriptions
	ros::Subscriber _subImu;
	ros::Subscriber _subGps;
	ros::Subscriber _subAirspeed;
	ros::Subscriber _subFlow;
	ros::Subscriber _subDistance;
	ros::Subscriber _subVision;
	ros::Subscriber _subMocap;
	ros::Subscriber _subLand;

	// publishers
	ros::Publisher _pubAttitude;
	ros::Publisher _pubLocalPosition;
	ros::Publisher _pubGlobalPosition;
	ros::Publisher _pubControlState;
	ros::Publisher _pubEstimatorStatus;
	ros::Publisher _pubInnov;

	// data
	Vector<float, X::n> _x0; 		// initial state vector
	Vector<float, X::n> _xMin; 		// lower bound vector
	Vector<float, X::n> _xMax; 		// upper bound vector
	Vector<float, Xe::n> _P0Diag; 	// initial state diagonal
	Vector<float, X::n> _x; 		// state vector
	SquareMatrix<float, Xe::n> _P; 	// covariance matrix
	Vector<float, U::n> _u; 		// input vector
	Vector3f _g_n; 					// expected gravity in navigation frame
	Origin _origin; 				// origin of local coordinate system
	float _baroAsl; 				// pressure altitude from baro
	float _baroOffset; 				// pressure offset
	bool _landed;					// are we landed?
	bool _freefall;					// are we freefalling?
	uint64_t _gpsUSec; 				// gps internal timestamp
	float _magDeclDeg; 				// magnetic declination (change in magnetic heading angle), deg
	bool _attitudeInitialized;		// attitude initializtion complete
	uint64_t _stateTimestamp;		// state prediction timestamp
	uint64_t _covarianceTimestamp;; // covariance prediction timestamp
	int _imuLowRateIndex;
	bool _accelSaturated;
	bool _gyroSaturated;
	SquareMatrix<float, Xe::n> _A;
	SquareMatrix<float, Xe::n> _Q;
	Vector<float, Xe::n> _dxe; 		// 	error vector
	SquareMatrix<float, Xe::n> _dP; 	// change in covariance matrix used for checking
	ekf2_innovations_s _innov;
};
