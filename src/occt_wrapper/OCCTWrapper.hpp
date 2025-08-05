
#ifndef occtwrapper_OCCTWrapper_hpp_
#define occtwrapper_OCCTWrapper_hpp_

#include <array>
#include <string>
#include <vector>
#include <utility>
#include <optional>

namespace Slic3r {

struct OCCTVolume {
    std::string volume_name;
    std::vector<std::array<float, 3>> vertices;
    std::vector<std::array<int, 3>> indices;
};

struct OCCTResult {
    std::string error_str;
    std::string object_name;
    std::vector<OCCTVolume> volumes;
};

using LoadStepFn = bool (*)(const char *path, OCCTResult* occt_result, std::optional<std::pair<double, double>> deflections);

}; // namespace Slic3r

#endif // occtwrapper_OCCTWrapper_hpp_
