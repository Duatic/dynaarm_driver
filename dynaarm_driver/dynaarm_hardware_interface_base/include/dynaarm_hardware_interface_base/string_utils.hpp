/*
 * Copyright 2024 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <string>
#include <string_view>
#include <algorithm>
#include <stdexcept>

namespace utils
{

/**
 * @brief Interpret a string as a bool
 * False, FALSE, false, 0 -> false
 * True, TRUE, 1 -> true
 *
 * @throw std::invalid_argument if string could not be parsed
 * @note create a copy of the passed string as it needs to transform it
 */
[[nodiscard]] inline bool sttobool(std::string s)
{
  // Transform to lower case
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);

  // Check for all the necessary cases
  if (s == "false" || s == "0") {
    return false;
  } else if (s == "true" || s == "1") {
    return true;
  }
  throw std::invalid_argument("Passed string could not be interpreted as bool: " + std::string(s));
}

}  // namespace utils