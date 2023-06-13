#include "load_obj.h"

#include <fstream>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <filesystem>

using namespace bvh::v2;

namespace obj {

    inline void remove_eol(char* ptr) {
        int i = 0;
        while (ptr[i]) i++;
        i--;
        while (i > 0 && std::isspace(ptr[i])) {
            ptr[i] = '\0';
            i--;
        }
    }

    inline char* strip_spaces(char* ptr) {
        while (std::isspace(*ptr)) ptr++;
        return ptr;
    }

    inline std::optional<int> read_index(char** ptr) {
        char* base = *ptr;

        // Detect end of line (negative indices are supported) 
        base = strip_spaces(base);
        if (!std::isdigit(*base) && *base != '-')
            return std::nullopt;

        int index = std::strtol(base, &base, 10);
        base = strip_spaces(base);

        if (*base == '/') {
            base++;

            // Handle the case when there is no texture coordinate
            if (*base != '/')
                std::strtol(base, &base, 10);

            base = strip_spaces(base);

            if (*base == '/') {
                base++;
                std::strtol(base, &base, 10);
            }
        }

        *ptr = base;
        return std::make_optional(index);
    }

    template <typename T>
    inline std::vector<Tri<T, 3>> load_from_stream(std::istream& is) {
        static constexpr size_t max_line = 1024;
        char line[max_line];

        std::vector<Vec<T, 3>> vertices;
        std::vector<Tri<T, 3>> triangles;

        while (is.getline(line, max_line)) {
            char* ptr = strip_spaces(line);
            if (*ptr == '\0' || *ptr == '#')
                continue;
            remove_eol(ptr);
            if (*ptr == 'v' && std::isspace(ptr[1])) {
                auto x = std::strtof(ptr + 1, &ptr);
                auto y = std::strtof(ptr, &ptr);
                auto z = std::strtof(ptr, &ptr);
                vertices.emplace_back(x, y, z);
            }
            else if (*ptr == 'f' && std::isspace(ptr[1])) {
                Vec<T, 3> points[2];
                ptr += 2;
                for (size_t i = 0; ; ++i) {
                    if (auto index = read_index(&ptr)) {
                        size_t j = *index < 0 ? vertices.size() + *index : *index - 1;
                        assert(j < vertices.size());
                        auto v = vertices[j];
                        if (i >= 2) {
                            triangles.emplace_back(points[0], points[1], v);
                            points[1] = v;
                        }
                        else {
                            points[i] = v;
                        }
                    }
                    else {
                        break;
                    }
                }
            }
        }

        return triangles;
    }



    template <typename T>
    inline std::vector<Tri<T,3>> load_stl_from_stream(std::ifstream& is){

        //Ignores header
        char header[80];
        is.read(header, 80);

        //Triangles
        std::vector<Tri<T, 3>> triangles;

        //Reads 4 bytes that specify the number of triangles
        uint32_t num_triangles;
        is.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));
        std::cout << "The number of triangles in the stl file is " << num_triangles << std::endl;

        // Loop over each triangle in the file
        while (is) {
            // read and ignore normal
            Vec<float, 3> normal;
            is.read(reinterpret_cast<char*>(&normal.values), sizeof(Vec<float, 3>));

            // Read all vertices
            for (int i = 0; i < num_triangles; i++) {
                Vec<T, 3> v1, v2, v3;
                is.read(reinterpret_cast<char*>(&v1), sizeof(Vec<T, 3>));
                is.read(reinterpret_cast<char*>(&v2), sizeof(Vec<T, 3>));
                is.read(reinterpret_cast<char*>(&v3), sizeof(Vec<T, 3>));
                triangles.emplace_back(v1, v2, v3);

                //Ignore attribute bytes
                uint16_t attribute_byte_count;
                is.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(uint16_t));
            }
        }

        std::cout << "Numbers of triangles " << triangles.size() << std::endl;
        return triangles;
    }

    template <typename T>
    inline std::vector<Tri<T, 3>> load_from_file(const std::string& file) {
        std::filesystem::path pathObj(file);
        std::string ext = pathObj.extension().string();
        if (ext == ".obj")
        {
            std::cout << "Loading file : " << file <<std::endl;
            std::ifstream is(file);
            if (is)
                return load_from_stream<T>(is);
            return std::vector<Tri<T, 3>>();
        }
        else if (ext == ".stl")
        {
            std::cout << "Loading file : " << file <<std::endl;
            std::ifstream is(file);
            if (is)
                return load_stl_from_stream<T>(is);
            return std::vector<Tri<T, 3>>();
        }
        else{
            std::cerr << "File format not supported" << std::endl;
        }
    }

    template <typename T>
    void print_triangles(const std::vector<Tri<T, 3>>& triangles) {
        for (const auto& tri : triangles) {
            std::cout << "p0: " << tri.p0 << "\n";
            std::cout << "p1: " << tri.p1 << "\n";
            std::cout << "p2: " << tri.p2 << "\n";
            std::cout << "-------------\n";
        }
    }

} // namespace obj


template <typename T>
std::vector<Tri<T, 3>> load_obj(const std::string& file) {
    auto tris =  obj::load_from_file<T>(file);
    return tris;
}


template std::vector<Tri<float, 3>> load_obj(const std::string&);
template std::vector<Tri<double, 3>> load_obj(const std::string&);
