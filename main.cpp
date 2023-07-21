#include <iostream>
#include "stl_header.h"
#include <string>
#include "include/tinynurbs/tinynurbs.h"
#include "simple_example.h"
#include "intersect/MeshIntersect.h"
#include "curvefitting/CurveFitting.h"
#include "nurbs/tests/catch.hpp"
#include "gcode/Printer.h"
#include "slice/Slice.h"
#include "simple_example.h"
#include "operations/Analysis.h"
//
// Created by simar on 6/12/2023.
//


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
    tinynurbs::Surface3f srf;
    srf.degree_u = 1;
    srf.degree_v = 1;
    srf.knots_u = {0, 0, 1, 1};
    srf.knots_v = {0, 0, 1, 1};
    // 2x2 grid (tinynurbs::array2) of control points
    srf.control_points = {2, 2,
                          {glm::vec3(-1, 0, 1), glm::vec3(-1, 0, -1),
                           glm::vec3(1, 0, 1), glm::vec3(1, 0, -1)
                          }
    };

    std::cout<< "Output" << std::to_string(surfaceNormal(srf,0.1f,0.1f)[0])<<std::endl;
   //tinynurbs::surfaceSaveOBJ(R"(C:\Code\SculptPlane\surface.obj)", srf);
}

void generateGCode(std::vector<MeshIntersect::intersection> intersections) {
    std::ofstream file(R"(C:\code\SP_NURBS_RTX\test_mp_2.gcode)");
    if (!file.is_open()) {
        std::cerr << "Failed to open output.gcode for writing." << std::endl;
        return;
    }

    // G-code header and initial setup
    file << "G28 ; Home all axes" << std::endl;
    file << "G21 ; Set units to millimeters" << std::endl;
    file << "G90 ; Use absolute coordinates" << std::endl;
    // Traverse the intersections
    for (int i = 0; i < intersections.size(); i++) {
        std::cout << "Iteration" << std::endl;
        std::cout << intersections[i];
        file << "G0 X" << intersections[i].intersectionPoint.values[0] + 150.0f
             << " Y" << intersections[i].intersectionPoint.values[1] + 150.0f
             << " Z" << intersections[i].intersectionPoint.values[2] << std::endl;
    }


    // G-code footer and ending commands
    file << "G0 Z10 ; Move the tool head up" << std::endl;
    file << "M84 ; Disable motors" << std::endl;

    file.close();
}

int main(){

//    Printer printer;
//    printer.setXSize(350);
//    printer.setYSize(350);
//    printer.setZSize(350);
//    printer.setNozzleSize(0.4);
//    printer.setMaxLayerHeight(0.2);
//    printer.setMinLayerHeight(0.1);
//    printer.setFilamentTemp(200);
//    printer.setBedTemp(60);
//    printer.setMinLayerWidth(0.4);
//    printer.setMaxLayerWidth(0.45);
//
//    Slice slice;
//    slice.beginSlice(R"(C:\code\SP_NURBS_RTX\test_mp_2.obj)", printer);
//
//    std::cout << "Welcome to Sculpt Path" << std::endl;
    std::string filePath = R"(C:\code\SP_NURBS_RTX\test_mp_2.obj)";
//    std::cout<<"Loading file : "<<filePath <<std::endl;
//    testNurbs();
//    //printTriangles(filePath);
//    //testMesh(filePath);
    MeshIntersect meshIntersect;
    meshIntersect.loadMesh(filePath);
//    //NURBSObj();
//    //meshIntersect.print_triangles();
//    meshIntersect.getMinMax(true);
//    std::vector<Vec3> intPoints = meshIntersect.planeIntersect(1.5f, true);
//    CurveFitting curveFitting;
//    tinynurbs::Curve<float> curv =  curveFitting.fitCurve(intPoints);
//    for (int i = 0; i < curv.control_points.size(); ++i) {
//        std::cout << "Control point number : " << std::to_string(i)<< std::endl;
//        std::cout << std::to_string(curv.control_points[i][0]) << std::endl;
//        std::cout << std::to_string(curv.control_points[i][1]) << std::endl;
//        std::cout << std::to_string(curv.control_points[i][2]) << std::endl;
//    }
//    std::cout << "Constructed curve knots : " << std::to_string(curv.knots.size()) << std::endl;
//    for (int i = 0; i < curv.knots.size(); ++i) {
//        std::cout << "Knot number : " << std::to_string(i)<< std::endl;
//        std::cout << "Knot value : " <<std::to_string(curv.knots[i]) << std::endl;
//    }


    auto ray = Ray {
            Vec3(0., 0.,0.3), // Ray origin
            Vec3(0., 0., 1.), // Ray direction
            0.,               // Minimum intersection distance
            50              // Maytxfximum intersection distance
    };
    Analysis analysis;
    analysis.performAnalysis(meshIntersect);
    //meshIntersect.perform_intersect(ray);
    //meshIntersect.getMinMax(true);
    //std::vector<std::vector<Vec3>> grid = meshIntersect.generateLinOvercastRayField(0.5f);
    //std::vector<MeshIntersect::intersection> intersectList = meshIntersect.gridPlaneIntersectMollerTrombore(grid);
    //std::cout << intersectList.size() << " Size of intersect list" << std::endl;
    //generateGCode(intersectList);
   // meshIntersect.getMinMax(true);

//    std::cout << "Welcome to Sculpt Path" << std::endl;
//    std::string filePath = R"(C:\Code\SculptPlane\test_mp_2.obj)";
//    std::cout<<"Loading file : "<<filePath <<std::endl;
//    testNurbs();
//    //printTriangles(filePath);
//    //testMesh(filePath);
//    MeshIntersect meshIntersect;
//    meshIntersect.loadMesh(filePath);
//    //NURBSObj();
//    //meshIntersect.print_triangles();

//    std::vector<Vec3> intPoints = meshIntersect.planeIntersect(1.5f, true);
//    CurveFitting curveFitting;
//    tinynurbs::Curve<float> curv =  curveFitting.fitCurve(intPoints);
//    for (int i = 0; i < curv.control_points.size(); ++i) {
//        std::cout << "Control point number : " << std::to_string(i)<< std::endl;
//        std::cout << std::to_string(curv.control_points[i][0]) << std::endl;
//        std::cout << std::to_string(curv.control_points[i][1]) << std::endl;
//        std::cout << std::to_string(curv.control_points[i][2]) << std::endl;
//    }
//    std::cout << "Constructed curve knots : " << std::to_string(curv.knots.size()) << std::endl;
//    for (int i = 0; i < curv.knots.size(); ++i) {
//        std::cout << "Knot number : " << std::to_string(i)<< std::endl;
//        std::cout << "Knot value : " <<std::to_string(curv.knots[i]) << std::endl;
//    }
//
//
   // testMesh(filePath, 5.0f, 5.0f);
    //testMesh(filePath, -5.0f, 1.0f);
   // testMesh(filePath, -5.0f, -1.0f);
   // testMesh(filePath, -5.0f, 0.5f);
    ray = Ray {
            Vec3(0., 5.,-3.), // Ray origin
            Vec3(0., 0., 1.), // Ray direction
            0.,               // Minimum intersection distance
            50.              // Maytxfximum intersection distance
    };
    //meshIntersect.perform_intersect(ray);

    ray = Ray {
            Vec3(0., 0.,-3.), // Ray origin
            Vec3(0., 0., 1.), // Ray direction
            0.,               // Minimum intersection distance
            50.              // Maximum intersection distance
    };
    //meshIntersect.perform_intersect(ray);

    ray = Ray {
            Vec3(0., 2,0.3), // Ray origin
            Vec3(0., 1., 0.), // Ray direction
            0.,               // Minimum intersection distance
            50.              // Maximum intersection distance
    };
    //meshIntersect.perform_intersect(ray);
}





