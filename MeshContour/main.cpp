// Suggestive Contour

//#include "halfedge_mesh.hpp"
#include <iostream>
#include <vector>
#include <numbers>
#include <fstream>
#include "mesh.hpp"
#include "contour.hpp"
#include "sphere.hpp"
#include "output.hpp"

void read_file(std::string const& filename, Mesh& mesh)
{
    std::ifstream ifs(filename);
    if (ifs.fail())
    {
        std::cerr << "Failed to open file." << "\n";
        std::exit(1);
    }
    std::string line;
    while (std::getline(ifs, line)){
        if (line.empty() || line[0] == '#')
        {
            // Do nothing
        }
        else if (line[0] == 'v')
        {
            Eigen::Vector3d v;
            std::istringstream string_in{line.substr(1)};
            string_in >> v(0) >> v(1) >> v(2);
            mesh.V.push_back(v);
        }
        else if (line[0] == 'f')
        {
            Eigen::Vector3i f;
            std::istringstream string_in{line.substr(1)};
            string_in >> f(0) >> f(1) >> f(2);
            f -= Eigen::Vector3i{1, 1, 1};
            mesh.F.push_back(f);
        }
    }
}

int main(int argc, const char * argv[])
{
    // 入力
    Eigen::Vector3d viewpoint;
    std::cout << "Enter a viewpoint: ";
    std::cin >> viewpoint(0) >> viewpoint(1) >> viewpoint(2);

    auto start = std::chrono::system_clock::now();

    // メッシュファイル読み込み
    std::string filename;
    if (argc != 2){
        std::cout << "command line error\n";
        std::exit(1);
    }
    filename = std::string(argv[1]);

    Mesh mesh;
    read_file(filename, mesh);
    mesh.make_halfedge_list();
    // 最小包含球が単位球となるようメッシュを正規化
    Sphere s = min_bounding_sphere(mesh);
    for (auto& v : mesh.V) {
        v = (v - s.center) / s.radius;
    }
    mesh.compute_normal();

    // Contourを生成
    Contour contour;
    std::vector<double> orientation;
    orientation = computeFaceOrientation(mesh, orientation, viewpoint);
    contour.generateMeshContour(mesh, orientation);
    // 出力
    outputMesh(mesh.V, mesh.F, filename);
    outputContour(contour.points, contour.lines, filename, "mesh_contour");
    outputFunctionValues(mesh, orientation, filename, "face_orientation");
//    outputViewpointPosition(viewpoint);
    openFile(filename);

    auto end = std::chrono::system_clock::now();
    using namespace std::chrono_literals;
    std::cerr << "実行時間 : " << (end - start) / 1.0s << " 秒\n";
}
