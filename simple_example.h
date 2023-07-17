#ifndef MESH_CALC_H
#define MESH_CALC_H

#include "mesh_calc/bvh.h"
#include "mesh_calc/vec.h"
#include "mesh_calc/ray.h"
#include "mesh_calc/node.h"
#include "mesh_calc/default_builder.h"
#include "mesh_calc/thread_pool.h"
#include "mesh_calc/executor.h"
#include "mesh_calc/stack.h"
#include "mesh_calc/tri.h"
#include "mesh_calc/load_obj.h"

#include <iostream>

using Scalar = float;
using Vec3 = bvh::v2::Vec<Scalar, 3>;
using BBox = bvh::v2::BBox<Scalar, 3>;
using Tri = bvh::v2::Tri<Scalar, 3>;
using Node = bvh::v2::Node<Scalar, 3>;
using Bvh = bvh::v2::Bvh<Node>;
using Ray = bvh::v2::Ray<Scalar, 3>;

using PrecomputedTri = bvh::v2::PrecomputedTri<Scalar>;

int testMesh(const std::string filePath, float d, float d1);

#endif // MESH_CALC_H