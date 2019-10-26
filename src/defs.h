#ifndef RAYTRACING_DEFS_H
#define RAYTRACING_DEFS_H

#include <GL/glew.h>
#include <Eigen/Eigen>

struct PrecomputedData {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector3f> normals;

    std::vector<Eigen::Vector3f> faceNormals;
    std::vector<std::vector<GLuint>> faceVertexIds;

    Eigen::Affine3f shapeModelMatrix;
};

#endif //RAYTRACING_DEFS_H
