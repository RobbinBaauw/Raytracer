#ifndef RAYTRACING_DEFS_H
#define RAYTRACING_DEFS_H

#include <GL/glew.h>
#include <Eigen/Eigen>

struct PrecomputedData {
    Eigen::Vector3f * vertices;
    Eigen::Vector3f * normals;

    Eigen::Vector3f * faceNormals;
    std::tuple<int, int, int> * faceVertexIds;

    Eigen::Affine3f shapeModelMatrix;
};

#endif //RAYTRACING_DEFS_H
