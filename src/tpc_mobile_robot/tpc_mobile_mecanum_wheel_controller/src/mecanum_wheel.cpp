/*********************************************************************
 * Software License Agreement (Apache License, Version 2.0 )
 *
 * Copyright 2024 TPC Mechatronics Crop.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

 /* Author: DeokLae Kim (kdl79@tanhay.com)*/

#include "tpc_mobile_mecanum_wheel_controller/mecanum_wheel.hpp"

namespace tpc
{
namespace tpc_mobile_mecanum_wheel_controller
{
    MecanumWheel::MecanumWheel(std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_position,
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_velocity,
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_velocity)
        : state_position_(state_position), state_velocity_(state_velocity), command_velocity_(command_velocity)
        {

        }
    void MecanumWheel::set_velocity(double value)
    {
      command_velocity_.get().set_value(value);
    }
}
}

