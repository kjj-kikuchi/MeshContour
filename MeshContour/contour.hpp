#ifndef contour_hpp
#define contour_hpp

#include <vector>
#include "mesh.hpp"

std::vector<double> computeFaceOrientation(Mesh const& mesh, std::vector<double>& orientation, Eigen::Vector3d const& viewpoint);

struct Contour
{
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2i> lines;

    void generateMeshContour(Mesh& mesh, std::vector<double> const& orientation);
};

#endif /* contour_hpp */
