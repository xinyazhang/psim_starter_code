#include "helper.h"
#include <igl/readOBJ.h>

namespace bird1 {

std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3i>
loadOBJ(const std::string& fn)
{
        Eigen::MatrixX3d V;
        Eigen::MatrixX3i F;
        igl::readOBJ(fn, V, F);
        return std::make_tuple(V, F);
}

}
