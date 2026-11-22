#pragma once

// Thin wrapper over Tracy so the rest of the code never includes it directly.
// With SIM_ENABLE_TRACY off, every macro compiles to nothing.

#ifdef SIM_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#define SIM_ZONE(literal_name) ZoneScopedN(literal_name)
#define SIM_ZONE_DYNAMIC(std_string) \
    ZoneScoped;                      \
    ZoneName((std_string).c_str(), (std_string).size())
#define SIM_FRAME_MARK(literal_name) FrameMarkNamed(literal_name)
#define SIM_PLOT(literal_name, value) TracyPlot(literal_name, value)
#else
#define SIM_ZONE(literal_name) ((void)0)
#define SIM_ZONE_DYNAMIC(std_string) ((void)0)
#define SIM_FRAME_MARK(literal_name) ((void)0)
#define SIM_PLOT(literal_name, value) ((void)0)
#endif
