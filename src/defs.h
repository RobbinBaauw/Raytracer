#ifndef RAYTRACING_DEFS_H
#define RAYTRACING_DEFS_H

#include <GL/glew.h>
#include <Eigen/Eigen>

const int MAXSOFTSHADOWPOINTS = 12;

struct PrecomputedData {
    Eigen::Vector3f * vertices;
    Eigen::Vector3f * normals;

    float * faceOriginDistance;
    Eigen::Vector3f * faceNormals;
    Eigen::Vector3f * faceNormalizedNormals;
    int * faceMaterialIds;
    std::tuple<int, int, int> * faceVertexIds;

    Eigen::Affine3f shapeModelMatrix;

    vector<vector<Eigen::Vector3f>> lights;
};

#endif //RAYTRACING_DEFS_H
