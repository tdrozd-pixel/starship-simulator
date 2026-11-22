#include "sim/server/NetUtil.hpp"

#include <cstdint>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#include <memory>
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

namespace sim::server {
namespace {

bool isUsable(uint32_t hostOrderAddr) {
    const uint32_t first = hostOrderAddr >> 24;
    const uint32_t second = (hostOrderAddr >> 16) & 0xFFu;
    if (first == 127 || first == 0) {
        return false;  // loopback
    }
    if (first == 169 && second == 254) {
        return false;  // APIPA / link-local
    }
    return true;
}

std::string format(uint32_t a) {
    return std::to_string(a >> 24) + "." + std::to_string((a >> 16) & 0xFFu) + "." +
           std::to_string((a >> 8) & 0xFFu) + "." + std::to_string(a & 0xFFu);
}

}  // namespace

std::vector<std::string> localIPv4Addresses() {
    std::vector<std::string> result;
#ifdef _WIN32
    ULONG size = 16 * 1024;
    std::unique_ptr<unsigned char[]> buffer;
    ULONG rc = ERROR_BUFFER_OVERFLOW;
    for (int attempt = 0; attempt < 3 && rc == ERROR_BUFFER_OVERFLOW; ++attempt) {
        buffer = std::make_unique<unsigned char[]>(size);
        rc = GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER,
                                  nullptr, reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.get()), &size);
    }
    if (rc != NO_ERROR) {
        return result;
    }
    for (auto* a = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.get()); a != nullptr; a = a->Next) {
        if (a->OperStatus != IfOperStatusUp) {
            continue;
        }
        for (auto* u = a->FirstUnicastAddress; u != nullptr; u = u->Next) {
            if (u->Address.lpSockaddr == nullptr || u->Address.lpSockaddr->sa_family != AF_INET) {
                continue;
            }
            const auto* sin = reinterpret_cast<const sockaddr_in*>(u->Address.lpSockaddr);
            const uint32_t addr = ntohl(sin->sin_addr.s_addr);
            if (isUsable(addr)) {
                result.push_back(format(addr));
            }
        }
    }
#else
    ifaddrs* list = nullptr;
    if (getifaddrs(&list) != 0) {
        return result;
    }
    for (const ifaddrs* a = list; a != nullptr; a = a->ifa_next) {
        if (a->ifa_addr == nullptr || a->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        const auto* sin = reinterpret_cast<const sockaddr_in*>(a->ifa_addr);
        const uint32_t addr = ntohl(sin->sin_addr.s_addr);
        if (isUsable(addr)) {
            result.push_back(format(addr));
        }
    }
    freeifaddrs(list);
#endif
    return result;
}

}  // namespace sim::server
