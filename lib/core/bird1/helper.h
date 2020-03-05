#ifndef PSIM_CORE_BIRD1_HELPER_H
#define PSIM_CORE_BIRD1_HELPER_H

#include <Eigen/Core>
#include <string>
#include <tuple>

namespace bird1 {

//
// loadOBJ: load OBJ file from fn and return the (V,F) tuple.
// 
// Note: We use "load" instead of "read" to resemble numpy.load, where in
//       python return value is commonly used instead of output parameter.
// 
std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3i>
loadOBJ(const std::string& fn);

}

#endif
