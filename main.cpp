#include <iostream>
#include "stl_header.h"
#include <string>
#include "include/tinynurbs/tinynurbs.h"
void printTriangles(const std::string filePath) {
    try{
        stl_reader::StlMesh<float, unsigned int> mesh (filePath);
        std::cout<< "The number of triangles : " << mesh.num_tris() <<std::endl;
        for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            std::cout << "coordinates of triangle " << itri << ": ";
            for(size_t icorner = 0; icorner < 3; ++icorner) {
                const float* c = mesh.tri_corner_coords (itri, icorner);
                // or alternatively:
                // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
                std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
            }
            std::cout << std::endl;

            float* n = const_cast<float *>(mesh.tri_normal(itri));
            std::cout   << "normal of triangle " << itri << ": "
                        << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
        }
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
}

void testNurbs(){
    tinynurbs::Curve<float> crv; // Planar curve using float32
    crv.control_points = {glm::vec3(-1, 0, 0), // std::vector of 3D points
                          glm::vec3(0, 1, 0),
                          glm::vec3(1, 0, 0)
    };
    crv.knots = {0, 0, 0, 1, 1, 1}; // std::vector of floats
    crv.degree = 2;
}


int main() {
    std::cout << "Welcome to Sculpt Path" << std::endl;
    std::string filePath = R"(C:\Code\SculptPlane\np_cube.stl)";
    std::cout<<"Loading file : "<<filePath <<std::endl;
    printTriangles(filePath);
    //testNurbs();

}



