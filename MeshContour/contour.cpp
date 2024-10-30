#include <iostream>
#include <vector>
#include <map>
#include "contour.hpp"

std::vector<double> computeFaceOrientation(Mesh const& mesh, std::vector<double>& orientation, Eigen::Vector3d const& viewpoint)
{
    orientation.resize(mesh.F.size());
    for (int i = 0; i < mesh.F.size(); i++) {
        orientation[i] = (viewpoint - mesh.V[mesh.F[i](0)]).dot(mesh.normalF[i]);
    }
    return orientation;
}

void Contour::generateMeshContour(Mesh& mesh, std::vector<double> const& orientation)
{
    Contour contour;
    std::map<std::tuple<double, double, double>, int> points_map;

    for (int i = 0; i < mesh.hEList.size(); i++)
    {
        int he_face = mesh.hEList[i].face();
        int he_opp_face = mesh.hEList[mesh.hEList[i].h_opp].face();
        if (orientation[he_face] * orientation[he_opp_face] < 0) {
            Eigen::Vector3d src = mesh.V[mesh.hEList[i].v_src()];
            Eigen::Vector3d tgt = mesh.V[mesh.hEList[i].v_tgt()];
            auto p0 = std::make_tuple(src(0), src(1), src(2));
            auto p1 = std::make_tuple(tgt(0), tgt(1), tgt(2));
            int p0_idx, p1_idx;
            if (points_map.contains(p0))
            {
                p0_idx = points_map.at(p0);
            }
            else
            {
                p0_idx = (int)points.size();
                points_map.emplace(p0, p0_idx);
                points.push_back(src);
            }
            if (points_map.contains(p1)){
                p1_idx = points_map.at(p1);
            }
            else
            {
                p1_idx = (int)points.size();
                points_map.emplace(p1, p1_idx);
                points.push_back(tgt);
            }
            lines.push_back(Eigen::Vector2i{p0_idx, p1_idx});
        }
    }
}
