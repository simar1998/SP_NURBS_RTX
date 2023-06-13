#pragma once

#include "tri.h"

#include <vector>
#include <string>

template <typename T>
std::vector<bvh::v2::Tri<T, 3>> load_obj(const std::string& file);

#include "tri.h"