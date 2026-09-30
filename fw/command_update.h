// Copyright 2026 Otavio Good.
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

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

/// Fork-specific: a command update (docs/protocol/registers.md, "Command
/// update").
///
/// An ordinary command starts with a mode write, and the controller resets
/// every command register to its default before the rest of the frame
/// writes what it carries.  A frame whose first byte is kCommandUpdate
/// instead changes only the registers it writes, keeps the rest of the
/// board's current command, and commands the result again.  It has no
/// effect while that command is "stopped" (after power-up or a STOP from
/// any source, the diagnostic console's "d stop" included), and a mode
/// write inside an update is refused with a write error, so an update can
/// never start a board: a host sends one full command, then only the
/// values that change.
namespace moteus {

constexpr uint8_t kCommandUpdate = 0x64;  // unused by mjlib

/// If the frame [data, *size) is a command update, remove the marker in
/// place and return true.
inline bool StripCommandUpdate(uint8_t* data, size_t* size) {
  if (*size < 1 || data[0] != kCommandUpdate) { return false; }
  std::memmove(data, data + 1, *size - 1);
  *size -= 1;
  return true;
}

}  // namespace moteus
