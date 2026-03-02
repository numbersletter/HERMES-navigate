// Copyright 2024 HERMES Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HERMES_NAVIGATE__COSTMAP_CONSTANTS_HPP_
#define HERMES_NAVIGATE__COSTMAP_CONSTANTS_HPP_

#include <cstdint>

namespace hermes_navigate
{
namespace costmap
{

/// Nav2 costmap cell value: completely free (traversable).
constexpr uint8_t FREE    = 0;

/// Nav2 costmap cell value: unknown / not yet observed.
constexpr uint8_t UNKNOWN = 255;

/// Nav2 costmap cell value: lethal obstacle (impassable).
constexpr uint8_t LETHAL  = 253;

/// Nav2 costmap cell value: inscribed lethal (robot centre would hit obstacle).
constexpr uint8_t INSCRIBED = 252;

}  // namespace costmap
}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__COSTMAP_CONSTANTS_HPP_
