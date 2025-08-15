/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#pragma once

namespace steered_diff_drive_controller
{

class PositionLimiter
{
public:
  /**
   * \brief Constructor
   * \param [in] has_position_limits     if true, applies position limits
   * \param [in] has_velocity_limits     if true, applies velocity limits
   * \param [in] has_accleration_limits  if true, applies acceleration limits
   * \param [in] min_position Minimum position [m], usually <= 0
   * \param [in] max_position Maximum position [m], usually >= 0
   * \param [in] min_velocity Minimum velocity [m/s], usually <= 0
   * \param [in] max_velocity Maximum velocity [m/s], usually >= 0
   * \param [in] min_acceleration Minimum acceleration [m/s^2], usually <= 0
   * \param [in] max_acceleration Maximum acceleration [m/s^2], usually >= 0
   */
  PositionLimiter(bool has_position_limits = false, bool has_velocity_limits = false, bool has_acceleration_limits = false,
               double min_position = 0.0, double max_position = 0.0, double min_velocity = 0.0,
               double max_velocity = 0.0, double min_acceleration = 0.0, double max_acceleration = 0.0);

  /**
   * \brief Limit the velocity and acceleration
   * \param [in, out] p  Position [m]
   * \param [in]      p0 Previous position to p  [m]
   * \param [in]      p1 Previous position to p0 [m]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double& p, double p0, double p1, double dt);

  /**
   * \brief Limit the position
   * \param [in, out] p Position [m]
   * \return Limiting factor (1.0 if none)
   */
  double limit_position(double& p);

  /**
   * \brief Limit the velocity
   * \param [in, out] p  Position [m/s]
   * \param [in]      p0 Previous position [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_velocity(double& p, double p0, double dt);

  /**
   * \brief Limit the acceleration
   * \param [in, out] p  Position [m/s]
   * \param [in]      p0 Previous position to p  [m/s]
   * \param [in]      p1 Previous position to p0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_acceleration(double& p, double p0, double p1, double dt);

public:
  // Enable/Disable position/velocity/acceleration limits:
  bool has_position_limits;
  bool has_velocity_limits;
  bool has_acceleration_limits;

  // Position limits:
  double min_position;
  double max_position;

  // Velocity limits:
  double min_velocity;
  double max_velocity;

  // Acceleration limits:
  double min_acceleration;
  double max_acceleration;
};

}  // namespace steered_diff_drive_controller
