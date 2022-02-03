/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file external_vision_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {
		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if (_params.fusion_mode & MASK_ROTATE_EV && !_control_status.flags.ev_yaw) {
			// update the rotation matrix which rotates EV measurements into the EKF's navigation frame
			// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
			const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
			_R_ev_to_ekf = Dcmf(q_error);

		} else {
			_R_ev_to_ekf.setIdentity();
		}

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_ev_sample_delayed.pos -= pos_offset_earth;


		controlEvPosFusion();
		controlEvVelFusion();
		controlEvYawFusion();

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = _ev_sample_delayed;
		_hpos_pred_prev = _state.pos;
		_hpos_prev_available = true;

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();

		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlEvPosFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVPOS) || _control_status.flags.ev_pos_fault) {
		stopEvPosFusion();
		return;
	}

	if (_control_status.flags.ev_pos) {
		if (_control_status_prev.flags.gps && !_control_status.flags.gps) {
			// GPS is no longer active
			resetHorizontalPositionToVision();
		}
	}

	if (_ev_data_ready) {

		// determine if we should use the horizontal position observations
		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;

		if (_control_status.flags.ev_pos) {

			// TODO: GPS <=> EV transitions
			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_pos && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample, reset vision position unless GPS is active
					if (!_control_status.flags.gps) {
						resetHorizontalPositionToVision();
					}

				} else {
					// Use an incremental position fusion method for EV position data if GPS is also used
					if (_control_status.flags.gps) {
						// GPS active
						fuseEvPositionDelta();

					} else {
						fuseEvPosition();
					}

					const bool is_fusion_failing = isTimedOut(_time_last_ev_pos_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_pos_reset_available > 0) {
							// Data seems good, attempt a reset
							resetHorizontalPosition();

							if (_control_status.flags.in_air) {
								_nb_ev_pos_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							_control_status.flags.ev_pos_fault = true;
							stopEvPosFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvPosFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvPosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV position fusion
				startEvPosFusion();

				if (_control_status.flags.ev_pos) {
					_nb_ev_pos_reset_available = 2;
				}
			}
		}

	} else if (_control_status.flags.ev_pos && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvPosFusion();
	}
}

void Ekf::controlEvVelFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVVEL) || _control_status.flags.ev_vel_fault) {
		stopEvVelFusion();
		return;
	}

	if (_ev_data_ready) {

		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;

		if (_control_status.flags.ev_vel) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_vel && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					resetVelocityToVision();

				} else {
					fuseEvVelocity();

					const bool is_fusion_failing = isTimedOut(_time_last_ev_vel_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_vel_reset_available > 0) {
							// Data seems good, attempt a reset
							resetVelocity();

							if (_control_status.flags.in_air) {
								_nb_ev_vel_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							_control_status.flags.ev_vel_fault = true;
							stopEvVelFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvVelFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvVelFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV velocity fusion
				startEvVelFusion();

				if (_control_status.flags.ev_vel) {
					_nb_ev_vel_reset_available = 2;
				}
			}
		}

	} else if (_control_status.flags.ev_vel && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvVelFusion();
	}
}

void Ekf::controlEvYawFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVYAW) || _control_status.flags.ev_vel_fault) {
		stopEvYawFusion();
		return;
	}

	if (_ev_data_ready) {

		// TODO: mag, gps yaw heading review?
		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)
				&& !_control_status.flags.gps && !_control_status.flags.gps_yaw;
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align; // TODO

		if (_control_status.flags.ev_yaw) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_yaw && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					resetYawToEv();

				} else {
					fuseEvYaw();

					const bool is_fusion_failing = isTimedOut(_time_last_ev_yaw_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_yaw_reset_available > 0) {
							// Data seems good, attempt a reset
							resetYawToEv();

							if (_control_status.flags.in_air) {
								_nb_ev_yaw_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							_control_status.flags.ev_yaw_fault = true;
							stopEvYawFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvYawFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV yaw fusion
				startEvYawFusion();

				if (_control_status.flags.ev_yaw) {
					_nb_ev_yaw_reset_available = 2;
				}
			}
		}

	} else if (_control_status.flags.ev_yaw && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvYawFusion();
	}
}

void Ekf::startEvPosFusion()
{
	if (!_control_status.flags.ev_pos) {
		_control_status.flags.ev_pos = true;

		// TODO: ev_pos vs delta
		if (!(_control_status.flags.gps && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us))) {
			resetHorizontalPositionToVision();
		}

		_information_events.flags.starting_vision_pos_fusion = true;
		ECL_INFO("starting vision pos fusion");
	}
}

void Ekf::startEvVelFusion()
{
	if (!_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = true;

		if (!(_control_status.flags.gps && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us))) {
			resetVelocityToVision();
		}

		_information_events.flags.starting_vision_vel_fusion = true;
		ECL_INFO("starting vision vel fusion");
	}
}

void Ekf::startEvYawFusion()
{
	if (!_control_status.flags.ev_yaw) {
		// turn on fusion of external vision yaw measurements and disable all magnetometer fusion
		_control_status.flags.ev_yaw = true;

		stopMagFusion();

		resetYawToEv();

		_control_status.flags.yaw_align = true;

		_information_events.flags.starting_vision_yaw_fusion = true;
		ECL_INFO("starting vision yaw fusion");
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		_control_status.flags.ev_pos = false;

		_ev_pos_innov.setZero();
		_ev_pos_innov_var.setZero();
		_ev_pos_test_ratio.setZero();

		_hpos_prev_available = false;
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = false;

		_ev_vel_innov.setZero();
		_ev_vel_innov_var.setZero();
		_ev_vel_test_ratio.setZero();
	}
}

void Ekf::stopEvYawFusion()
{
	if (_control_status.flags.ev_yaw) {
		_control_status.flags.ev_yaw = false;

		_heading_innov = 0.f;
		_heading_innov_var = 0.f;
		_yaw_test_ratio = 0.f;
	}
}

void Ekf::fuseEvPosition()
{
	Vector3f ev_pos_meas = _R_ev_to_ekf * _ev_sample_delayed.pos;
	Matrix3f ev_pos_var = _R_ev_to_ekf * matrix::diag(_ev_sample_delayed.posVar) * _R_ev_to_ekf.transpose();

	_ev_pos_innov(0) = _state.pos(0) - ev_pos_meas(0);
	_ev_pos_innov(1) = _state.pos(1) - ev_pos_meas(1);
	_ev_pos_innov(2) = _state.pos(2) - ev_pos_meas(2);

	Vector3f obs_var;
	obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.01f));
	obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.01f));
	obs_var(2) = fmaxf(ev_pos_var(2, 2), sq(0.01f));

	// innovation gate size
	const float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	_ev_pos_innov(0) = P(7, 7) + obs_var(0);
	_ev_pos_innov(1) = P(8, 8) + obs_var(1);
	_ev_pos_innov(2) = P(9, 9) + obs_var(2);

	float test_ratio_x = sq(_ev_pos_innov(0)) / (sq(innov_gate) * _ev_pos_innov_var(0));
	float test_ratio_y = sq(_ev_pos_innov(1)) / (sq(innov_gate) * _ev_pos_innov_var(1));
	//float test_ratio_z = sq(_ev_pos_innov(2)) / (sq(innov_gate) * _ev_pos_innov_var(2));

	_ev_pos_test_ratio(0) = fmaxf(test_ratio_x, test_ratio_y);
	//_ev_pos_test_ratio(1) = test_ratio_z;

	if (_ev_pos_test_ratio(0) <= 1.f) {
		// innovation check passed
		_innov_check_fail_status.flags.reject_ev_pos_x = false;
		_innov_check_fail_status.flags.reject_ev_pos_y = false;

		_fault_status.flags.bad_ev_pos_x = !fuseVelPosHeight(_ev_pos_innov(0), _ev_pos_innov_var(0), 3);
		_fault_status.flags.bad_ev_pos_y = !fuseVelPosHeight(_ev_pos_innov(1), _ev_pos_innov_var(1), 4);

		if (!_fault_status.flags.bad_ev_pos_x && !_fault_status.flags.bad_ev_pos_y) {
			_time_last_ev_pos_fuse = _time_last_imu;
			_time_last_hor_pos_fuse = _time_last_imu;
		}

	} else {
		_innov_check_fail_status.flags.reject_ev_pos_x = true;
		_innov_check_fail_status.flags.reject_ev_pos_y = true;
	}
}

void Ekf::fuseEvPositionDelta()
{
	if (_hpos_prev_available) {
		// calculate the change in position since the last measurement
		// rotate measurement into body frame is required when fusing with GPS
		Vector3f ev_delta_pos = _R_ev_to_ekf * Vector3f(_ev_sample_delayed.pos - _ev_sample_delayed_prev.pos);

		// use the change in position since the last measurement
		_ev_pos_innov(0) = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
		_ev_pos_innov(1) = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);
		_ev_pos_innov(2) = _state.pos(2) - _hpos_pred_prev(2) - ev_delta_pos(2);

		// observation 1-STD error, incremental pos observation is expected to have more uncertainty
		Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
		ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();

		Vector3f obs_var;
		obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
		obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));
		obs_var(2) = fmaxf(ev_pos_var(2, 2), sq(0.5f));

		// innovation gate size
		const float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

		_ev_pos_innov(0) = P(7, 7) + obs_var(0);
		_ev_pos_innov(1) = P(8, 8) + obs_var(1);
		_ev_pos_innov(2) = P(9, 9) + obs_var(2);

		float test_ratio_x = sq(_ev_pos_innov(0)) / (sq(innov_gate) * _ev_pos_innov_var(0));
		float test_ratio_y = sq(_ev_pos_innov(1)) / (sq(innov_gate) * _ev_pos_innov_var(1));
		float test_ratio_z = sq(_ev_pos_innov(2)) / (sq(innov_gate) * _ev_pos_innov_var(2));

		_ev_pos_test_ratio(0) = fmaxf(test_ratio_x, test_ratio_y);
		_ev_pos_test_ratio(1) = test_ratio_z;

		if (_ev_pos_test_ratio(0) <= 1.f) {
			// innovation check passed
			_innov_check_fail_status.flags.reject_ev_pos_x = false;
			_innov_check_fail_status.flags.reject_ev_pos_y = false;

			_fault_status.flags.bad_ev_pos_x = !fuseVelPosHeight(_ev_pos_innov(0), _ev_pos_innov_var(0), 3);
			_fault_status.flags.bad_ev_pos_y = !fuseVelPosHeight(_ev_pos_innov(1), _ev_pos_innov_var(1), 4);

			if (!_fault_status.flags.bad_ev_pos_x && !_fault_status.flags.bad_ev_pos_y) {
				_time_last_ev_pos_fuse = _time_last_imu;
				_time_last_delpos_fuse = _time_last_imu;
			}

		} else {
			_innov_check_fail_status.flags.reject_ev_pos_x = true;
			_innov_check_fail_status.flags.reject_ev_pos_y = true;
		}


		// z
		bool innov_check_pass = (_ev_pos_test_ratio(2) <= 1.f);

		// if there is bad vertical acceleration data, then don't reject measurement,
		// but limit innovation to prevent spikes that could destabilise the filter
		float innovation = _ev_pos_innov(2);

		if (_fault_status.flags.bad_acc_vertical && !innov_check_pass) {
			const float innov_limit = innov_gate * sqrtf(_ev_pos_innov_var(2));
			innovation = math::constrain(_ev_pos_innov(2), -innov_limit, innov_limit);
			innov_check_pass = true;
		}

		if (innov_check_pass) {
			// innovation check passed
			_innov_check_fail_status.flags.reject_ev_pos_z = false;
			_fault_status.flags.bad_ev_pos_z = !fuseVelPosHeight(innovation, _ev_pos_innov_var(2), 5);

		} else {
			_innov_check_fail_status.flags.reject_ev_pos_z = true;
		}
	}
}

void Ekf::fuseEvVelocity()
{
	const Vector3f obs_var = matrix::max(getVisionVelocityVarianceInEkfFrame(), sq(0.05f));
	const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

	_ev_vel_innov = _state.vel - getVisionVelocityInEkfFrame();

	_ev_vel_innov_var(0) = P(4, 4) + obs_var(0);
	_ev_vel_innov_var(1) = P(5, 5) + obs_var(1);
	_ev_vel_innov_var(2) = P(6, 6) + obs_var(2);

	float test_ratio_x = sq(_ev_vel_innov(0)) / (sq(innov_gate) * _ev_vel_innov_var(0));
	float test_ratio_y = sq(_ev_vel_innov(1)) / (sq(innov_gate) * _ev_vel_innov_var(1));
	float test_ratio_z = sq(_ev_vel_innov(2)) / (sq(innov_gate) * _ev_vel_innov_var(2));

	_ev_vel_test_ratio(0) = fmaxf(test_ratio_x, test_ratio_y);
	_ev_vel_test_ratio(1) = test_ratio_z;

	// vx & vy
	if (_ev_vel_test_ratio(0) <= 1.f) {
		// innovation check passed
		_innov_check_fail_status.flags.reject_ev_vel_x = false;
		_innov_check_fail_status.flags.reject_ev_vel_y = false;

		_fault_status.flags.bad_ev_vel_x = !fuseVelPosHeight(_ev_vel_innov(0), _ev_vel_innov_var(0), 0);
		_fault_status.flags.bad_ev_vel_y = !fuseVelPosHeight(_ev_vel_innov(1), _ev_vel_innov_var(1), 1);

		if (!_fault_status.flags.bad_ev_vel_x && !_fault_status.flags.bad_ev_vel_y) {
			_time_last_ev_vel_fuse = _time_last_imu;
			_time_last_hor_vel_fuse = _time_last_imu;
		}

	} else {
		_innov_check_fail_status.flags.reject_ev_vel_x = true;
		_innov_check_fail_status.flags.reject_ev_vel_y = true;
	}

	// vz
	bool innov_check_pass = (_ev_vel_test_ratio(1) <= 1.0f);

	// if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	float innovation = _ev_vel_innov(2);

	if (_fault_status.flags.bad_acc_vertical && !innov_check_pass) {
		const float innov_limit = innov_gate * sqrtf(_ev_vel_innov_var(2));
		innovation = math::constrain(_ev_vel_innov(2), -innov_limit, innov_limit);
		innov_check_pass = true;
	}

	if (innov_check_pass) {
		_innov_check_fail_status.flags.reject_ev_vel_z = false;

		_fault_status.flags.bad_ev_vel_z = !fuseVelPosHeight(innovation, _ev_vel_innov_var(2), 2);

		if (!_fault_status.flags.bad_ev_vel_z) {
			_time_last_ev_vel_fuse = _time_last_imu;
			_time_last_ver_vel_fuse = _time_last_imu;
		}

		// used by checkVerticalAccelerationHealth
		_vert_vel_innov_ratio = _ev_vel_innov(2) / sqrtf(_ev_vel_innov_var(2));
		_vert_vel_fuse_time_us = _time_last_imu;

	} else {
		_innov_check_fail_status.flags.reject_ev_vel_z = true;
	}
}

void Ekf::fuseEvYaw()
{
	if (shouldUse321RotationSequence(_R_to_earth)) {
		float measured_hdg = getEuler321Yaw(_ev_sample_delayed.quat);

		if (fuseYaw321(measured_hdg, _ev_sample_delayed.angVar)) {
			_time_last_ev_yaw_fuse = _time_last_imu;
			_fault_status.flags.bad_ev_yaw = true;

		} else {
			_fault_status.flags.bad_ev_yaw = true;
		}

	} else {
		float measured_hdg = getEuler312Yaw(_ev_sample_delayed.quat);

		if (fuseYaw312(measured_hdg, _ev_sample_delayed.angVar)) {
			_time_last_ev_yaw_fuse = _time_last_imu;
			_fault_status.flags.bad_ev_yaw = true;

		} else {
			_fault_status.flags.bad_ev_yaw = true;
		}
	}
}
