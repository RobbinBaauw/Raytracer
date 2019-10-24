#include "flyscene.hpp"
#include <GLFW/glfw3.h>

void Flyscene::initialize(int width, int height) {
    // initiliaze the Phong Shading effect for the Opengl Previewer
    phong.initialize();

    // set the camera's projection matrix
    flycamera.setPerspectiveMatrix(60.0, (float) width / (float) height, 0.1f, 100.0f);
    flycamera.setViewport(Eigen::Vector2f((float) width, (float) height));

    // load the OBJ file and materials
//    Tucano::MeshImporter::loadObjFile(mesh, materials,"resources/models/dodgeColorTest.obj");
    Tucano::MeshImporter::loadObjFile(mesh, materials, "resources/models/bunny.obj");

  // set the camera's projection matrix
  flycamera.setPerspectiveMatrix(60.0, width / (float)height, 0.1f, 100.0f);
  flycamera.setViewport(Eigen::Vector2f((float)width, (float)height));

  // load the OBJ file and materials
  Tucano::MeshImporter::loadObjFile(mesh, materials,
                                    "resources/models/plane.obj");


  mesh.normalizeModelMatrix();

  // pass all the materials to the Phong Shader
  for (int i = 0; i < materials.size(); ++i)
    phong.addMaterial(materials[i]);

  // set the color and size of the sphere to represent the light sources
  // same sphere is used for all sources
  lightrep.setColor(Eigen::Vector4f(1.0, 1.0, 0.0, 1.0));
  lightrep.setSize(0.15);

  // create a first ray-tracing light source at some random position
  lights.push_back(Eigen::Vector3f(-0.5, 2.0, 3.0));

  // scale the camera representation (frustum) for the ray debug
  camerarep.shapeMatrix()->scale(0.2);

  // the debug ray is a cylinder, set the radius and length of the cylinder
  ray.setSize(0.005, 1.0);

  // craete a first debug ray pointing at the center of the screen
  createDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

  glEnable(GL_DEPTH_TEST);

  // for (int i = 0; i<mesh.getNumberOfFaces(); ++i){
  //   Tucano::Face face = mesh.getFace(i);    
  //   for (int j =0; j<face.vertex_ids.size(); ++j){
  //     std::cout<<"vid "<<j<<" "<<face.vertex_ids[j]<<std::endl;
  //     std::cout<<"vertex "<<mesh.getVertex(face.vertex_ids[j]).transpose()<<std::endl;
  //     std::cout<<"normal "<<mesh.getNormal(face.vertex_ids[j]).transpose()<<std::endl;
  //   }
  //   std::cout<<"mat id "<<face.material_id<<std::endl<<std::endl;
  //   std::cout<<"face   normal "<<face.normal.transpose() << std::endl << std::endl;
  // }



}

void Flyscene::paintGL(void) {

  // update the camera view matrix with the last mouse interactions
  flycamera.updateViewMatrix();
  Eigen::Vector4f viewport = flycamera.getViewport();

  // clear the screen and set background color
  glClearColor(0.9, 0.9, 0.9, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // position the scene light at the last ray-tracing light source
  scene_light.resetViewMatrix();
  scene_light.viewMatrix()->translate(-lights.back());

  // render the scene using OpenGL and one light source
  phong.render(mesh, flycamera, scene_light);

  // render the ray and camera representation for ray debug
  ray.render(flycamera, scene_light);
  camerarep.render(flycamera, scene_light);

  // render ray tracing light sources as yellow spheres
  for (int i = 0; i < lights.size(); ++i) {
    lightrep.resetModelMatrix();
    lightrep.modelMatrix()->translate(lights[i]);
    lightrep.render(flycamera, scene_light);
  }

  // render coordinate system at lower right corner
  flycamera.renderAtCorner();
}

void Flyscene::simulate(GLFWwindow *window) {
  // Update the camera.
  // NOTE(mickvangelderen): GLFW 3.2 has a problem on ubuntu where some key
  // events are repeated: https://github.com/glfw/glfw/issues/747. Sucks.
  float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 1.0 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 1.0 : 0.0);
  float dy = (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS
                  ? 1.0
                  : 0.0) -
             (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS
                  ? 1.0
                  : 0.0);
  float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 1.0 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 1.0 : 0.0);
  flycamera.translate(dx, dy, dz);
}

Eigen::Vector3f Flyscene::shadeOffFace(int faceIndex, const Eigen::Vector3f& origin, const Eigen::Vector3f& hitPosition) {
    Tucano::Face face = mesh.getFace(faceIndex);
    int materialIndex = face.material_id;
    Tucano::Material::Mtl material = materials[materialIndex];

    Eigen::Vector3f lightIntensity = Eigen::Vector3f(1, 1, 1);

    // Interpolating the normal
    const auto currVertexIds = face.vertex_ids;

    // TODO add drawing of why this way of calculating is logical
    const Eigen::Vector3f v0 = (mesh.getShapeMatrix() * mesh.getModelMatrix() * mesh.getVertex(currVertexIds[0])).head(3);
    const Eigen::Vector3f v1 = (mesh.getShapeMatrix() * mesh.getModelMatrix() * mesh.getVertex(currVertexIds[1])).head(3);
    const Eigen::Vector3f v2 = (mesh.getShapeMatrix() * mesh.getModelMatrix() * mesh.getVertex(currVertexIds[2])).head(3);

    const auto v0Normal = mesh.getNormal(currVertexIds[0]).normalized();
    const auto v1Normal = mesh.getNormal(currVertexIds[1]).normalized();
    const auto v2Normal = mesh.getNormal(currVertexIds[2]).normalized();

    const float faceArea = (v1 - v0).cross(v2 - v0).norm() * 0.5;

    const auto areaV1V2Hitpoint = (v1 - hitPosition).cross(v2 - hitPosition).norm() * 0.5;
    const auto areaV0V2Hitpoint = (v0 - hitPosition).cross(v2 - hitPosition).norm() * 0.5;
    const auto areaV0V1Hitpoint = (v0 - hitPosition).cross(v1 - hitPosition).norm() * 0.5;

    Eigen::Vector3f faceNormal = areaV1V2Hitpoint * v0Normal + areaV0V2Hitpoint * v1Normal + areaV0V1Hitpoint * v2Normal;
    faceNormal = faceNormal / faceArea;
    faceNormal.normalize();

    Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);

    // Iterate over all the present lights
    for (const Eigen::Vector3f& lightPosition : lights) {

        Eigen::Vector3f lightDirection = (lightPosition - hitPosition).normalized();

        // Diffuse term
        float cos1 = fmaxf(0, lightDirection.dot(faceNormal));
        Eigen::Vector3f diffuse = lightIntensity.cwiseProduct(material.getDiffuse()) * cos1;

        std::cout << "Diffuse in: " << material.getDiffuse() << std::endl;
        std::cout << "Diffuse out: " << diffuse << std::endl;

        // Specular term
        Eigen::Vector3f eyeDirection = (origin - hitPosition).normalized();
        Eigen::Vector3f reflectedLight = (-lightDirection + 2.f * lightDirection.dot(faceNormal) * faceNormal);
        reflectedLight.normalize();

        float cos2 = fmax(0, reflectedLight.dot(eyeDirection));
        Eigen::Vector3f specular = lightIntensity.cwiseProduct(material.getSpecular()) * (pow(cos2, material.getShininess()));

        std::cout << "Specular in: " << material.getSpecular() << std::endl;
        std::cout << "Specular out: " << specular << std::endl;

        const auto colorSum = diffuse + specular;
        Eigen::Vector3f minSum = colorSum.cwiseMax(0.0).cwiseMin(1.0);
        minSum[3] = material.getOpticalDensity();

        color += minSum;

        std::cout << "Light color out: " << color << std::endl;
    }

    return color;
}

const int MAXRECURSION = 2;

bool Flyscene::intersectsPlane(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction,
        const Eigen::Vector3f& normal, const Eigen::Vector3f& v0,
        float& currMaxDepth, Eigen::Vector3f& hitPoint) {

    // Get distance from triangle to origin (see slide 27)
    const auto originDistance = normal.dot(v0);

    // Compute tHit (see slide 10)
    const auto tHit = (originDistance - origin.dot(normal)) / (direction.dot(normal));

    if (tHit < 0.00001f || tHit >= currMaxDepth) {
        return false;
    }

    // Compute hit point (see slide 10)
    hitPoint = origin + tHit * direction;
    currMaxDepth = tHit;

    return true;
}

bool Flyscene::intersectsTriangle(const Eigen::Vector3f& origin, const Eigen::Vector3f& dest, const Eigen::Vector3f& direction, Eigen::Vector3f& hitPoint, int& hitPointFaceId) {
    bool hasIntersected = false;
    float currentMaxDepth = std::numeric_limits<float>::max();

    auto nrOfFaces = mesh.getNumberOfFaces();
    for (int i = 0; i < nrOfFaces; i++) {

        // Retrieve the current face and its vertex ids
        const auto currFace = mesh.getFace(i);
        const auto currVertexIds = currFace.vertex_ids;

        assert(currVertexIds.size() == 3);

        // Create the vertices
        const Eigen::Vector3f v0 = (mesh.getShapeMatrix() * mesh.getModelMatrix() * mesh.getVertex(currVertexIds[0])).head(3);
        const Eigen::Vector3f v1 = (mesh.getShapeMatrix() * mesh.getModelMatrix() * mesh.getVertex(currVertexIds[1])).head(3);
        const Eigen::Vector3f v2 = (mesh.getShapeMatrix() * mesh.getModelMatrix() * mesh.getVertex(currVertexIds[2])).head(3);

        // Get normal (slide 26)
        const auto normal = ((v1 - v0).cross(v2 - v0)).normalized();

        Eigen::Vector3f currentHitPoint;
        if (!intersectsPlane(origin, direction, normal, v0, currentMaxDepth, currentHitPoint)) {
            return false;
        }

        const float denom = normal.dot(normal);

        const auto edge0 = v1 - v0;
        const auto edge1 = v2 - v1;
        const auto edge2 = v0 - v2;

        const float a = normal.dot(edge1.cross(currentHitPoint - v1)) / denom;
        const float b = normal.dot(edge2.cross(currentHitPoint - v2)) / denom;

        // No need to calculate "c" as it's (1 - a - b)
        if (a > 0.0 && b > 0.0 && a + b < 1.000001) {
            hasIntersected = true;

            hitPointFaceId = i;
            hitPoint = currentHitPoint;
        }
    }
  }

    return hasIntersected;
}

Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f &origin, Eigen::Vector3f &dest, int recursionDepth) {

    // Get direction of ray
    const auto direction = (dest - origin).normalized();

    // References if intersects
    Eigen::Vector3f hitPoint;
    int hitPointFaceId;

    if (intersectsTriangle(origin, dest, direction, hitPoint, hitPointFaceId)) {
        const Eigen::Vector3f localShading = shadeOffFace(hitPointFaceId, origin, hitPoint);

        std::cout << "Intersection, depth = " << recursionDepth << std::endl;

        if (recursionDepth < MAXRECURSION) {
            Tucano::Face face = mesh.getFace(hitPointFaceId);
            Eigen::Vector3f faceNormal = face.normal.normalized();
            int materialIndex = face.material_id;
            Tucano::Material::Mtl material = materials[materialIndex];

            const auto &specular = material.getSpecular();
            float EPSILON = 0.00001f;
            if (specular.x() > EPSILON || specular.y() > EPSILON || specular.z() > EPSILON) {

                // Reflection
                const auto hitFace = mesh.getFace(hitPointFaceId);
                const auto normal = hitFace.normal;
                Eigen::Vector3f reflection = direction - 2 * direction.dot(normal) * normal;
                const Eigen::Vector3f reflectionShading = traceRay(hitPoint, reflection, recursionDepth + 1);
                const Eigen::Vector3f weightedReflectionShading = {
                reflectionShading.x() * specular.x(),
                reflectionShading.y() * specular.y(),
                reflectionShading.z() * specular.z()
                };

                // Refraction
                Eigen::Vector3f refraction = { 0, 0, 0 };
                const Eigen::Vector3f refractionShading = traceRay(hitPoint, refraction, recursionDepth + 1);
                const Eigen::Vector3f weightedRefractionShading = {
                refractionShading.x() * (1 - specular.x()),
                refractionShading.y() * (1 - specular.y()),
                refractionShading.z() * (1 - specular.z())
                };

                return {
                localShading.x() + weightedReflectionShading.x() + weightedRefractionShading.x(),
                localShading.y() + weightedReflectionShading.y() + weightedRefractionShading.y(),
                localShading.z() + weightedReflectionShading.z() + weightedRefractionShading.z()
                };
            }
        }

        return localShading;
    } else {
        std::cout << "No intersection, depth = " << recursionDepth << std::endl;

        return {
        0.0,
        0.0,
        0.0
        };
    }
}


    // for every pixel shoot a ray from the origin through the pixel coords
    for (int y = startY; y < startY + amountY; ++y) {
        for (int x = 0; x < image_size[0]; ++x) {
            // create a ray from the camera passing through the pixel (i,j)
            Eigen::Vector3f screen_coords = flycamera.screenToWorld(Eigen::Vector2f(x, y));

            // launch raytracing for the given ray and write result to pixel data
            const Eigen::Vector3f &colorOut = traceRay(origin, screen_coords, 0);

            pixel_data[x][y] = colorOut;
        }
    }
}
