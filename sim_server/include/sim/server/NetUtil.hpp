#pragma once

#include <string>
#include <vector>

namespace sim::server {

// Non-loopback, non-link-local IPv4 addresses of this machine, for printing LAN URLs.
std::vector<std::string> localIPv4Addresses();

}  // namespace sim::server
