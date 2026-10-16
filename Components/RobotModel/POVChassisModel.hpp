/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_POV_CHASSIS_MODEL_HPP
#define FINEMOTE_POV_CHASSIS_MODEL_HPP

#include "URDF/finemote.hpp"
#include "pov_chassis_urdf.hpp"

namespace finemote::pov {

inline constexpr auto kConfig = fineurdf::build_config_default(pov_chassis_LITERAL);
static_assert(kConfig.ok, "Invalid POV URDF: inspect kConfig.err_text");
static_assert(kConfig.motor_count == 8 && kConfig.module_count == 4,
              "POV example requires four steering/drive pairs");

} // namespace finemote::pov

#endif // FINEMOTE_POV_CHASSIS_MODEL_HPP
