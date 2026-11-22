#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <string_view>

namespace sim {

// Derive an independent seed for a named random stream from the master seed,
// so adding a new model never changes the noise seen by existing ones.
inline uint64_t deriveSeed(uint64_t masterSeed, std::string_view streamName) {
    uint64_t h = 1469598103934665603ULL;  // FNV-1a
    for (char c : streamName) {
        h ^= static_cast<unsigned char>(c);
        h *= 1099511628211ULL;
    }
    uint64_t z = masterSeed ^ h;  // splitmix64 finalizer
    z += 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// Deterministic across compilers and platforms.
// std::mt19937_64 is fully specified by the standard, but std::normal_distribution
// and std::uniform_real_distribution are not, so MSVC and GCC would produce
// different flights from the same seed. Distributions are implemented here instead.
class Rng {
public:
    explicit Rng(uint64_t seed = 1) : engine_(seed) {}

    // Uniform in [0, 1)
    double uniform() { return static_cast<double>(engine_() >> 11) * 0x1.0p-53; }

    // Standard normal (Marsaglia polar method)
    double normal() {
        if (hasSpare_) {
            hasSpare_ = false;
            return spare_;
        }
        double u = 0.0;
        double v = 0.0;
        double s = 0.0;
        do {
            u = 2.0 * uniform() - 1.0;
            v = 2.0 * uniform() - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);
        const double m = std::sqrt(-2.0 * std::log(s) / s);
        spare_ = v * m;
        hasSpare_ = true;
        return u * m;
    }

    double normal(double mean, double sigma) { return mean + sigma * normal(); }

private:
    std::mt19937_64 engine_;
    double spare_ = 0.0;
    bool hasSpare_ = false;
};

}  // namespace sim
