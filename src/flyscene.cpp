#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include <cassert>
#include <thread>
#include <chrono>
#include "boundingBox.h"

//#define HARDSHADOW

#define TIMESTAMPING
//#define DETAILTIMESTAMPING
//#define LOGGING

const int MAXRECURSION = 5;
const int MAXDEBUGRECURSION = 10;

void Flyscene::initialize(int width, int height) {
#ifdef TIMESTAMPING
    std::cout << "Initializing scene ..." << std::endl;

    std::chrono::time_point<std::chrono::steady_clock> completeStart = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> end;
    std::chrono::milliseconds diff;
#endif

    // initiliaze the Phong Shading effect for the Opengl Previewer
    phong.initialize();

    // set the camera's projection matrix
    flycamera.setPerspectiveMatrix(60.0, (float) width / (float) height, 0.1f, 100.0f);
    flycamera.setViewport(Eigen::Vector2f((float) width, (float) height));

    // load the OBJ file and materials
    Tucano::MeshImporter::loadObjFile(mesh, materials, "resources/models/bunny.obj");

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Loading obj: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    // normalize the model (scale to unit cube and center at origin)
    mesh.normalizeModelMatrix();


  mesh.normalizeModelMatrix();

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Init p1: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Debug ray: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    camerarep.resetModelMatrix();
    camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Init p2: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    precomputeData();
    precomputeLights();

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Precomputing: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    std::vector<Eigen::Vector3f> boundaries = boundingVectors();
    boxMain = boundingBox(boundaries[0], boundaries[1]);

    int numb_faces = mesh.getNumberOfFaces();
    for (int i = 0; i < numb_faces; ++i) {
        boxMain.addFaceIndex(i);
    }

    boxMain.splitBox(precomputedData);
    boxMain.computeDepth();

    startDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Bounding boxes: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

#ifdef LOGGING
    std::cout << "Node count is: " << boundingBox::getNode() << std::endl;
    std::cout << "Leaf count is: " << boundingBox::getLeaf() << std::endl;

    std::vector<boundingBox> currChildren = boxMain.getChildren();
    int depth = 0;
    while (!currChildren.empty()) {
        ++depth;
        currChildren = currChildren[0].getChildren();
    }
    std::cout << "Depth of the lefmost branch is " << depth << std::endl;
#endif

    // craete a first debug ray pointing at the center of the screen
    createDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

    glEnable(GL_DEPTH_TEST);
}

void Flyscene::precomputeData() {

    precomputedData.shapeModelMatrix = mesh.getShapeModelMatrix();

    precomputedData.normals = new Eigen::Vector3f[mesh.getNumberOfVertices()];
    precomputedData.vertices = new Eigen::Vector3f[mesh.getNumberOfVertices()];

    precomputedData.faceOriginDistance = new float[mesh.getNumberOfFaces()];
    precomputedData.faceNormals = new Eigen::Vector3f[mesh.getNumberOfFaces()];
    precomputedData.faceNormalizedNormals = new Eigen::Vector3f[mesh.getNumberOfFaces()];
    precomputedData.faceMaterialIds = new int[mesh.getNumberOfFaces()];
    precomputedData.faceVertexIds = new std::tuple<int, int, int>[mesh.getNumberOfFaces()];

    auto nrOfFaces = mesh.getNumberOfFaces();
    for (int i = 0; i < nrOfFaces; i++) {

        const auto currFace = mesh.getFace(i);
        const auto currVertexIds = currFace.vertex_ids;

        // v0
        const auto v0Id = currVertexIds[0];
        const Eigen::Vector3f v0 = (precomputedData.shapeModelMatrix * mesh.getVertex(v0Id)).head(3);
        precomputedData.vertices[v0Id] = v0;

        const auto v0Normal = mesh.getNormal(v0Id).normalized();
        precomputedData.normals[v0Id] = v0Normal;

        // v1
        const auto v1Id = currVertexIds[1];
        const Eigen::Vector3f v1 = (precomputedData.shapeModelMatrix * mesh.getVertex(v1Id)).head(3);
        precomputedData.vertices[v1Id] = v1;

        const auto v1Normal = mesh.getNormal(v1Id).normalized();
        precomputedData.normals[v1Id] = v1Normal;

        // v2
        const auto v2Id = currVertexIds[2];
        const Eigen::Vector3f v2 = (precomputedData.shapeModelMatrix * mesh.getVertex(v2Id)).head(3);
        precomputedData.vertices[v2Id] = v2;

        const auto v2Normal = mesh.getNormal(v2Id).normalized();
        precomputedData.normals[v2Id] = v2Normal;

        // Faces
        precomputedData.faceOriginDistance[i] = currFace.normal.dot(v0);
        precomputedData.faceNormals[i] = currFace.normal;
        precomputedData.faceNormalizedNormals[i] = currFace.normal.normalized();
        precomputedData.faceMaterialIds[i] = currFace.material_id;
        precomputedData.faceVertexIds[i] = {
                currVertexIds[0],
                currVertexIds[1],
                currVertexIds[2]
        };
    }
}

void Flyscene::precomputeLights() {
    precomputedData.lights = vector<vector<Eigen::Vector3f>>(lights.size());

#ifdef HARDSHADOW
    for (unsigned int i = 0; i < lights.size(); i++) {
        const auto &light = lights[i];
        vector<Eigen::Vector3f> lightPositions({light});
        precomputedData.lights[i] = lightPositions;
    }
#else
    const float radius = 0.3;

    for (unsigned int i = 0; i < lights.size(); i++) {

        const auto &light = lights[i];

        vector<Eigen::Vector3f> lightPositions(MAXSOFTSHADOWPOINTS);
        for (int n = 0; n < MAXSOFTSHADOWPOINTS; n++) {
            float theta = (2.0f * (float) n * (float) M_PI) / MAXSOFTSHADOWPOINTS;
            float phi = acos(1 - 2 * ((float) n / (MAXSOFTSHADOWPOINTS * (float) M_PI)));
            float x = radius * sin(phi) * cos(theta);
            float y = radius * sin(phi) * sin(theta);
            float z = radius * cos(phi);
            lightPositions[n] = Eigen::Vector3f(light[0] + x, light[1] + y, light[2] + z);
        }

        precomputedData.lights[i] = lightPositions;
    }
#endif
}

void Flyscene::paintGL() {

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


    if (splitPreviewDepth != -1) {
        boxMain.renderLeafBoxes(flycamera, scene_light, renderIntersection, splitPreviewDepth, 0);
    }

    // render the scene using OpenGL and one light source
    phong.render(mesh, flycamera, scene_light);

    camerarep.render(flycamera, scene_light);

    // render ray tracing light sources as yellow spheres
    for (const auto &light : lights) {
        lightrep.resetModelMatrix();
        lightrep.modelMatrix()->translate(light);
        lightrep.render(flycamera, scene_light);
    }

}

void Flyscene::paintGL(void) {

    float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 0.6f : 0.4f) -
               (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 0.6f : 0.4f);

    float dy = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS ? 0.6f : 0.4f) -
               (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ? 0.6f : 0.4f);

    float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 0.6f : 0.4f) -
               (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 0.6f : 0.4f);

    flycamera.translate(dx, dy, dz);
}

void Flyscene::startDebugRay(const Eigen::Vector2f &mouseCoords) {

    rays.clear();

    // from pixel position to world coordinates
    flycamera.reComputeViewMatrix();
    Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouseCoords);

    std::vector<int> indices;
    const Eigen::Vector3f &origin = flycamera.getCenter();
    boxMain.intersectingBoxes(origin, screen_pos, indices);

    // direction from camera center to click position
    Eigen::Vector3f dir = (screen_pos - origin).normalized();

    createDebugRay(origin, dir, 0);
}

void Flyscene::createDebugRay(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, int recursionDepth) {

    auto currentRay = Tucano::Shapes::Cylinder(0.1, 1.0, 16, 64);

    // the debug ray is a cylinder, set the radius and length of the cylinder
    currentRay.setSize(0.005, 10.0);
    currentRay.resetModelMatrix();

    // position and orient the cylinder representing the ray
    currentRay.setOriginOrientation(origin, direction);

    Eigen::Vector3f hitPoint;
    int faceId;

    Eigen::Vector3f reflection;
    Eigen::Vector3f refraction;

    if (intersects(origin, direction, faceId, hitPoint, reflection, refraction) && recursionDepth < MAXDEBUGRECURSION) {

        const auto lengthRay = (origin - hitPoint).norm();
        currentRay.setSize(0.005, lengthRay);

        // Reflection
        reflection.normalize();
        createDebugRay(hitPoint, reflection, recursionDepth + 1);

//        // Refraction
//        refraction.normalize();
//        createDebugRay(hitPoint, refraction, recursionDepth + 1);
    }

    rays.emplace_back(currentRay);
}

void Flyscene::raytraceScene() {

#ifdef TIMESTAMPING
    std::cout << "-------------------------" << std::endl;
    std::cout << "Starting ray tracing ..." << std::endl;

    std::chrono::time_point<std::chrono::steady_clock> completeStart = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> end;
    std::chrono::milliseconds diff;
#endif

    // if no width or height passed, use dimensions of current viewport
    Eigen::Vector2i image_size = flycamera.getViewportSize();

    // create 2d vector to hold pixel colors and resize to match image size
    vector<vector<Eigen::Vector3f>> pixel_data;
    int &xSize = image_size[0];
    int &ySize = image_size[1];
    pixel_data.resize(ySize);
    for (int i = 0; i < ySize; ++i)
        pixel_data[i].resize(image_size[0]);

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Initialization stuff: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    // origin of the ray is always the camera center
    Eigen::Vector3f origin = flycamera.getCenter();

    const auto threadCount = thread::hardware_concurrency();
//    const auto threadCount = 1;

    vector<thread> threads;

    for (int threadIndex = 0; threadIndex < threadCount; threadIndex++) {
        threads.emplace_back(&Flyscene::tracePixels, this, threadIndex, threadCount, std::ref(origin), std::ref(pixel_data), std::ref(image_size)); // Requires std::ref
    }

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Dividing threads: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    for (int i = 0; i < threadCount; i++) {
        threads[i].join();
    }

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Raytracing: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    // write the ray tracing result to a PPM image
    Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Writing file: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

#ifdef TIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - completeStart);
    std::cout << "Done! Total time: " << diff.count() << "ms" << std::endl;
#endif
}

Eigen::Vector3f Flyscene::shadeOffFace(int faceIndex, const Eigen::Vector3f &origin, const Eigen::Vector3f &hitPosition) {

    Eigen::Vector3f color = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    int &materialIndex = precomputedData.faceMaterialIds[faceIndex];
    if (materialIndex == -1) {
        return {0.5f, 0.5f, 0.5f};
    }

    Eigen::Vector3f lightIntensity = getLightIntensity(hitPosition);

    Tucano::Material::Mtl &material = materials[materialIndex];

    // Interpolating the normal
    const auto &currVertexIds = precomputedData.faceVertexIds[faceIndex];

    // TODO add drawing of why this way of calculating is logical
    const Eigen::Vector3f &v0 = precomputedData.vertices[get<0>(currVertexIds)];
    const Eigen::Vector3f &v1 = precomputedData.vertices[get<1>(currVertexIds)];
    const Eigen::Vector3f &v2 = precomputedData.vertices[get<2>(currVertexIds)];

    const Eigen::Vector3f &v0Normal = precomputedData.normals[get<0>(currVertexIds)];
    const Eigen::Vector3f &v1Normal = precomputedData.normals[get<1>(currVertexIds)];
    const Eigen::Vector3f &v2Normal = precomputedData.normals[get<2>(currVertexIds)];

    const auto areaV1V2Hitpoint = (v1 - hitPosition).cross(v2 - hitPosition).norm() * 0.5;
    const auto areaV0V2Hitpoint = (v0 - hitPosition).cross(v2 - hitPosition).norm() * 0.5;
    const auto areaV0V1Hitpoint = (v0 - hitPosition).cross(v1 - hitPosition).norm() * 0.5;

    Eigen::Vector3f faceNormal = areaV1V2Hitpoint * v0Normal + areaV0V2Hitpoint * v1Normal + areaV0V1Hitpoint * v2Normal;
    faceNormal = faceNormal / faceArea;
    faceNormal.normalize();

    // Iterate over all the present lights
    for (const Eigen::Vector3f &lightPosition : lights) {

			// Ambient term

			Eigen::Vector3f lightDirection = (lightPosition - hitPosition).normalized();
			const Eigen::Vector3f ambient = lightIntensity.cwiseProduct(material.getAmbient());

			// Diffuse term
			float cos1 = fmaxf(0, lightDirection.dot(faceNormal));
			const Eigen::Vector3f diffuse = lightIntensity.cwiseProduct(material.getDiffuse()) * cos1;

			// Specular term
			const Eigen::Vector3f eyeDirection = (origin - hitPosition).normalized();
			const Eigen::Vector3f reflectedLight = (-lightDirection + 2.f * lightDirection.dot(faceNormal) * faceNormal).normalized();

			float cos2 = fmax(0, reflectedLight.dot(eyeDirection));
			Eigen::Vector3f specular = lightIntensity.cwiseProduct(material.getSpecular()) * (pow(cos2, material.getShininess()));


			const auto colorSum = diffuse + specular + ambient;
			const auto colorSumMinMax = colorSum.cwiseMax(0.0).cwiseMin(1.0);
			//        Eigen::Vector4f minSum = {
			//                colorSumMinMax.x(),
			//                colorSumMinMax.y(),
			//                colorSumMinMax.z(),
			//                0.0f
			//        };
			//        minSum[3] = material.getOpticalDensity();

			color += colorSumMinMax;
		}
	//}
	//else
	//{
		//color = Eigen::Vector3f(0,0,0);
		//std::cout << "SHADOW" << std::endl;
	//}

#ifdef LOGGING
    std::cout << "Color" << std::endl;
    std::cout << color << std::endl;
#endif

    return color;
}

Eigen::Vector3f Flyscene::traceRay(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, int recursionDepth) {

    Eigen::Vector3f hitPoint;
    int faceId;

    Eigen::Vector3f reflection;
    Eigen::Vector3f refraction;

#ifdef DETAILTIMESTAMPING
    std::chrono::time_point<std::chrono::steady_clock> completeStart = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> end;
    std::chrono::microseconds diff;
#endif

    bool b = intersects(origin, direction, faceId, hitPoint, reflection, refraction);

#ifdef DETAILTIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Intersection check: " << diff.count() << "us" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    if (b) {
        const Eigen::Vector3f localShading = shadeOffFace(faceId, origin, hitPoint);

#ifdef DETAILTIMESTAMPING
        end = std::chrono::steady_clock::now();
        diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Shading: " << diff.count() << "us" << std::endl;
        start = std::chrono::steady_clock::now();
#endif

        if (recursionDepth < MAXRECURSION) {
            int &materialIndex = precomputedData.faceMaterialIds[faceId];
            if (materialIndex == -1) {
                return localShading;
            }

            Tucano::Material::Mtl &material = materials[materialIndex];

            const auto &specular = material.getSpecular();
            float EPSILON = 0.01f;
            if (specular.x() > EPSILON || specular.y() > EPSILON || specular.z() > EPSILON) {

                // Reflection
                reflection.normalize();

                const Eigen::Vector3f reflectionShading = traceRay(hitPoint + 0.01f * reflection, reflection, recursionDepth + 1);
                const Eigen::Vector3f weightedReflectionShading = {
                reflectionShading.x() * specular.x(),
                reflectionShading.y() * specular.y(),
                reflectionShading.z() * specular.z()
                };

                // Refraction
                refraction.normalize();

//                const Eigen::Vector3f refractionShading = traceRay(hitPoint, refraction, recursionDepth + 1);
//                const Eigen::Vector3f weightedRefractionShading = {
//                        refractionShading.x() * specular.x(),
//                        refractionShading.y() * specular.y(),
//                        refractionShading.z() * specular.z()
//                };

                return {
                        localShading.x() + weightedReflectionShading.x(), // + weightedRefractionShading.x(),
                        localShading.y() + weightedReflectionShading.y(), // + weightedRefractionShading.y(),
                        localShading.z() + weightedReflectionShading.z(), // + weightedRefractionShading.z()
                };
            }
        }

        return localShading;
    } else {

        // Background color
        if (recursionDepth == 0) {
            return {
                    0.7,
                    0.9,
                    0.9
            };
        }

        return {
                0.0,
                0.0,
                0.0
        };
    }
}

bool Flyscene::intersects(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                          int &faceId, Eigen::Vector3f &hitpoint,
                          Eigen::Vector3f &reflection, Eigen::Vector3f &refraction) {

    float currentMaxDepth = numeric_limits<float>::max();

    const bool intersectsSphere = sphereIntersection(currentMaxDepth, origin, direction, faceId, hitpoint, reflection, refraction);
    const bool intersectsTriangle = triangleIntersection(currentMaxDepth, origin, direction, faceId, hitpoint, reflection, refraction);

    return intersectsSphere || intersectsTriangle;
}

bool Flyscene::sphereIntersection(float &currentMaxDepth, const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                                  int &faceId, Eigen::Vector3f &hitpoint,
                                  Eigen::Vector3f &reflection, Eigen::Vector3f &refraction) {

    bool hasIntersected = false;

    // TODO finish, note that the currentMaxDepth should be the same measurement as in intersectTriangle
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection :)

//    for (auto &sphere : spheres) {
//        const Eigen::Vector3f &originToSphere = sphere.center - origin;
//        const float distOriginSphere = originToSphere.norm();
//
//        float a = direction.norm();
//        float b = 2.0f * originToSphere.dot(direction);
//        float c = distOriginSphere - sphere.radius * sphere.radius;
//
//        float D = b * b - 4 * a * c;
//
//        if (D < 1 || distOriginSphere >= currentMaxDepth) continue;
//
//        hasIntersected = true;
//        currentMaxDepth = distOriginSphere;
//
//        const auto DSquare = sqrt(D);
//        const auto aSummed = a + a;
//
//        const auto solution1 = (-b + DSquare) / aSummed;
//        const auto solution2 = (-b - DSquare) / aSummed;
//
//        reflection = direction - 2 * direction.dot(normal) * normal;
//        refraction = Eigen::Vector3f(0, 0, 0); // TODO
//
//        hitpoint = hitPoint;
//        faceId = -1;
//    }

    return hasIntersected;
}

bool Flyscene::triangleIntersection(float &currentMaxDepth, const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                                    int &faceId, Eigen::Vector3f &hitpoint,
                                    Eigen::Vector3f &reflection, Eigen::Vector3f &refraction) {

#ifdef DETAILTIMESTAMPING
    std::chrono::time_point<std::chrono::steady_clock> completeStart = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> end;
    std::chrono::microseconds diff;
#endif

    vector<int> intersectingFaces;
    boxMain.intersectingBoxes(origin, direction, intersectingFaces);

#ifdef DETAILTIMESTAMPING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "DS check: " << diff.count() << "us, " << intersectingFaces.size() << " faces left" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    bool hasIntersected = false;

    for (auto &i : intersectingFaces) {
        const Eigen::Vector3f &normal = precomputedData.faceNormals[i];

        // Get distance from triangle to origin (see slide 27)
        const auto &originDistance = precomputedData.faceOriginDistance[i];

        // Compute tHit (see slide 10)
        const auto tHit = (originDistance - origin.dot(normal)) / (direction.dot(normal));

        if (tHit < 0.00001f || tHit > currentMaxDepth) continue;

        // Retrieve the current face and its vertex ids
        const auto &currVertexIds = precomputedData.faceVertexIds[i];

        // Create the vertices
        const Eigen::Vector3f &v0 = precomputedData.vertices[get<0>(currVertexIds)];
        const Eigen::Vector3f &v1 = precomputedData.vertices[get<1>(currVertexIds)];
        const Eigen::Vector3f &v2 = precomputedData.vertices[get<2>(currVertexIds)];

        // Compute hit point (see slide 10)
        const auto hitPoint = origin + tHit * direction;

        const auto a = (v1 - v0).cross(hitPoint - v0).dot(normal);
        if (a < 0) continue;

        const auto b = (v2 - v1).cross(hitPoint - v1).dot(normal);
        if (b < 0) continue;

        const auto c = (v0 - v2).cross(hitPoint - v2).dot(normal);
        if (c < 0) continue;

        hasIntersected = true;
        currentMaxDepth = tHit;

        reflection = direction - 2 * direction.dot(normal) * normal;
        refraction = Eigen::Vector3f(0, 0, 0); // TODO

        hitpoint = hitPoint;
        faceId = i;
    }
    return hasIntersected;
}

void Flyscene::tracePixels(int threadId, int threads, Eigen::Vector3f &origin, vector<vector<Eigen::Vector3f>> &pixel_data,
                           Eigen::Vector2i &image_size) {

    const int &xSize = image_size[0];
    const int &ySize = image_size[1];

    flycamera.reComputeViewMatrix();

    for (int x = 0; x < xSize; x++) {
        for (int y = threadId; y < ySize; y += threads) {
            // create a ray from the camera passing through the pixel (i,j)
            const Eigen::Vector3f &screen_coords = flycamera.screenToWorld(Eigen::Vector2f(x, y));

            const Eigen::Vector3f direction = (screen_coords - origin).normalized();

            // launch raytracing for the given ray and write result to pixel data
            const Eigen::Vector3f &colorOut = traceRay(origin, direction, 0);

            pixel_data[y][x] = colorOut;
        }
    }
}

std::vector<Eigen::Vector3f> Flyscene::boundingVectors() {
    int num_verts = mesh.getNumberOfVertices();

    const Eigen::Vector3f &firstVertex = precomputedData.vertices[0];
    Eigen::Vector3f vmin = {
            firstVertex.x(),
            firstVertex.y(),
            firstVertex.z()
    };

    Eigen::Vector3f vmax = {
            firstVertex.x(),
            firstVertex.y(),
            firstVertex.z()
    };
    for (int i = 0; i < num_verts; ++i) {
        const Eigen::Vector3f &vertex = precomputedData.vertices[i];
        vmin(0) = min(vmin.x(), vertex.x());
        vmax(0) = max(vmax.x(), vertex.x());

        vmin(1) = min(vmin.y(), vertex.y());
        vmax(1) = max(vmax.y(), vertex.y());

        vmin(2) = min(vmin.z(), vertex.z());
        vmax(2) = max(vmax.z(), vertex.z());
    }
    return {vmin, vmax};
}

Eigen::Vector3f Flyscene::getLightIntensity(const Eigen::Vector3f &hitPosition) {

    float lightIntensity = 0;

    Eigen::Vector3f hitPoint;
    int faceId;
    Eigen::Vector3f reflection;
    Eigen::Vector3f refraction;

    // For each light
    for (const auto &lightPositions : precomputedData.lights) {
        float pointsReachingLight = 0;

        for (const Eigen::Vector3f &lightPosition : lightPositions) {
            if (!intersects(hitPosition, lightPosition, faceId, hitPoint, reflection, refraction)) {
                pointsReachingLight++;
            }
        }

        lightIntensity += pointsReachingLight / lightPositions.size();
    }

    // Calculate the light intensity and return
    float maximum = 1;
    lightIntensity = min(lightIntensity, maximum);
    return {lightIntensity, lightIntensity, lightIntensity};
}
