#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include <cassert>
#include <thread>
#include <chrono>

#define LOGGING

void Flyscene::initialize(int width, int height) {
    // initiliaze the Phong Shading effect for the Opengl Previewer
    phong.initialize();

    // set the camera's projection matrix
    flycamera.setPerspectiveMatrix(60.0, (float) width / (float) height, 0.1f, 100.0f);
    flycamera.setViewport(Eigen::Vector2f((float) width, (float) height));

    // load the OBJ file and materials
//    Tucano::MeshImporter::loadObjFile(mesh, materials,"resources/models/dodgeColorTest.obj");
    Tucano::MeshImporter::loadObjFile(mesh, materials, "resources/models/cube.obj");


    // normalize the model (scale to unit cube and center at origin)
    mesh.normalizeModelMatrix();

    // pass all the materials to the Phong Shader
    for (auto &material : materials)
        phong.addMaterial(material);

    // set the color and size of the sphere to represent the light sources
    // same sphere is used for all sources
    lightrep.setColor(Eigen::Vector4f(1.0, 1.0, 0.0, 1.0));
    lightrep.setSize(0.15);

    // create a first ray-tracing light source at some random position
    lights.emplace_back(-1.0, 1.0, 1.0);

    // scale the camera representation (frustum) for the ray debug
    camerarep.shapeMatrix()->scale(0.2);

    // the debug ray is a cylinder, set the radius and length of the cylinder
    ray.setSize(0.005, 10.0);

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

void Flyscene::paintGL() {

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
    for (const auto &light : lights) {
        lightrep.resetModelMatrix();
        lightrep.modelMatrix()->translate(light);
        lightrep.render(flycamera, scene_light);
    }

    // render coordinate system at lower right corner
    flycamera.renderAtCorner();
}

void Flyscene::simulate(GLFWwindow *window) {
    // Update the camera.
    // NOTE(mickvangelderen): GLFW 3.2 has a problem on ubuntu where some key
    // events are repeated: https://github.com/glfw/glfw/issues/747. Sucks.

    float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 0.6f : 0.4f) -
               (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 0.6f : 0.4f);

    float dy = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS ? 0.6f : 0.4f) -
               (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ? 0.6f : 0.4f);

    float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 0.6f : 0.4f) -
               (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 0.6f : 0.4f);

    flycamera.translate(dx, dy, dz);
}

void Flyscene::createDebugRay(const Eigen::Vector2f &mouse_pos) {
    ray.resetModelMatrix();
    // from pixel position to world coordinates
    Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);

    // direction from camera center to click position
    Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();

    // position and orient the cylinder representing the ray
    ray.setOriginOrientation(flycamera.getCenter(), dir);

    // place the camera representation (frustum) on current camera location,
    camerarep.resetModelMatrix();
    camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());
}

void Flyscene::raytraceScene(int width, int height) {

#ifdef LOGGING
    std::cout << "Starting ray tracing ..." << std::endl;

    std::chrono::time_point<std::chrono::steady_clock> completeStart = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> end;
    std::chrono::milliseconds diff;
#endif

    // if no width or height passed, use dimensions of current viewport
    Eigen::Vector2i image_size(width, height);
    if (width == 0 || height == 0) {
        image_size = flycamera.getViewportSize();
    }

    // create 2d vector to hold pixel colors and resize to match image size
    vector<vector<Eigen::Vector3f>> pixel_data;
    int ySize = image_size[1];
    pixel_data.resize(ySize);
    for (int i = 0; i < ySize; ++i)
        pixel_data[i].resize(image_size[0]);

#ifdef LOGGING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Initialization stuff: " << diff.count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
#endif

    // origin of the ray is always the camera center
    Eigen::Vector3f origin = flycamera.getCenter();

//    const auto threadCount = thread::hardware_concurrency();
    const auto threadCount = 1;
    const auto ysPerThread = ceil((float) ySize / (float) threadCount); // the amount of y coordinates per thread, ceil. Meaning we start with this number and if it is divisble by the remaining threads we do -1

    vector<thread> threads;

    auto startingY = 0;
    for (int i = 0; i < threadCount; i++) {
        const auto threadsLeft = threadCount - i;
        const auto ysToDo = ySize - startingY;

        const int currentYs = ysToDo % threadsLeft == 0 ? ysToDo / threadsLeft : ysPerThread;

        threads.emplace_back(&Flyscene::traceFromY, this, startingY, currentYs, std::ref(origin), std::ref(pixel_data), std::ref(image_size));

        startingY += currentYs;
    }

    for (int i = 0; i < threadCount; i++) {
        threads[i].join();
    }

#ifdef LOGGING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Writing file: " << diff.count() << "ms" << std::endl;
#endif

    // write the ray tracing result to a PPM image
    Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);

#ifdef LOGGING
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - completeStart);
    std::cout << "Done! Total time: " << diff.count() << "ms" << std::endl;
#endif
}

Eigen::Vector3f Flyscene::shadeOffFace(int faceIndex, const Eigen::Vector3f& rayDirection) {
    Tucano::Face face = mesh.getFace(faceIndex);
    int materialIndex = face.material_id;
    Tucano::Material::Mtl material = materials[materialIndex];

    Eigen::Vector3f faceNormal = face.normal.normalized();

    Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);

    // Iterate over all the present lights
    for (Eigen::Vector3f lightPosition : lights) {

        //Compute average location face
        Eigen::Vector3f avgLoc = ((
                  mesh.getVertex(face.vertex_ids[0]) +
                  mesh.getVertex(face.vertex_ids[1]) +
                  mesh.getVertex(face.vertex_ids[2])
          ) / 3).head(3);

        lightPosition.normalize();
        Eigen::Vector3f lightDirection = (lightPosition - avgLoc).normalized();
        Eigen::Vector3f lightIntensity = Eigen::Vector3f(0.5, 0.5, 0.5);

        // Ambient term
        Eigen::Vector3f ambient = lightIntensity.cwiseProduct(material.getAmbient());

        std::cout << "Ambient in: " << material.getAmbient().x() << ", " << material.getAmbient().y() << ", " << material.getAmbient().z() << std::endl;
        std::cout << "Ambient out: " << ambient.x() << ", " << ambient.y() << ", " << ambient.z() << std::endl;

        // Diffuse term
        float cos1 = fmaxf(0, lightDirection.dot(faceNormal));
        Eigen::Vector3f diffuse = lightIntensity.cwiseProduct(material.getDiffuse()) * cos1;

        std::cout << "Diffuse in: " << material.getDiffuse().x() << ", " << material.getDiffuse().y() << ", " << material.getDiffuse().z() << std::endl;
        std::cout << "Diffuse out: " << diffuse.x() << ", " << diffuse.y() << ", " << diffuse.z() << std::endl;

        // Specular term
//        Eigen::Vector3f reflectedLight = lightDirection - 2.f * lightDirection.dot(faceNormal) * faceNormal;
//        float cos2 = fmax(0, reflectedLight.dot(eyeDirection));
//        Eigen::Vector3f specular = lightIntensity.cwiseProduct(material.getSpecular()) * (pow(cos2, material.getShininess()));

        color += ambient + diffuse;

        std::cout << "Color out: " << color.x() << ", " << color.y() << ", " << color.z() << std::endl;
    }

    return color;
}

const int MAXRECURSION = 1;

Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f &origin, Eigen::Vector3f &dest, int recursionDepth) {

    bool hasIntersected = false;
    float currentMaxDepth = std::numeric_limits<float>::max();

    Eigen::Vector3f currentHitpoint;
    int currentMaxDepthFaceId = -1;

    // Get direction of ray
    const auto direction = (dest - origin).normalized();

    auto nrOfFaces = mesh.getNumberOfFaces();
    for (int i = 0; i < nrOfFaces; i++) {
        // Retrieve the current face and its vertex ids
        const auto currFace = mesh.getFace(i);
        const auto currVertexIds = currFace.vertex_ids;

        assert(currVertexIds.size() == 3);

        // Create the vertices
        const auto v0 = (mesh.getShapeMatrix() * mesh.getVertex(currVertexIds[0])).head(3);
        const auto v1 = (mesh.getShapeMatrix() * mesh.getVertex(currVertexIds[1])).head(3);
        const auto v2 = (mesh.getShapeMatrix() * mesh.getVertex(currVertexIds[2])).head(3);

        // Get normal (implemented by Tucano)
        const auto normal = currFace.normal;

        // Get distance from triangle to origin (see slide 27)
        const auto originDistance = normal.dot(v0);

        // Compute tHit (see slide 10)
        const auto tHit = (originDistance - origin.dot(normal)) / (direction.dot(normal));

        // Compute hit point (see slide 10)
        const auto hitPoint = origin + tHit * direction;

        // We want to solve p = v2 + a * (v0 - v2) + b * (v1 - v2), see slide 28
        // This we can do by getting the linear combination for (p - v2), thus giving us a and b
        // So we solve Ax = b where A exists of (v0 - v2), (v1, v2) and b exists of (p - v2)
        Eigen::Matrix<float, 3, 2> A;
        A << (v0 - v2), (v1 - v2);

        const auto linearCombination = A.colPivHouseholderQr().solve(hitPoint - v2);
        const auto a = linearCombination[0];
        const auto b = linearCombination[1];

        auto noHit = a < 0 || b < 0 || a + b > 1 || tHit < 0.00001f;
        if (!noHit && tHit < currentMaxDepth) {
            hasIntersected = true;
            currentMaxDepth = tHit;

            currentHitpoint = hitPoint;
            currentMaxDepthFaceId = i;
        }
    }

    if (hasIntersected) {
        const Eigen::Vector3f localShading = shadeOffFace(currentMaxDepthFaceId, direction);

        std::cout << "Intersection, depth = " << recursionDepth << std::endl;

        if (recursionDepth < MAXRECURSION) {
            Tucano::Face face = mesh.getFace(currentMaxDepthFaceId);
            Eigen::Vector3f faceNormal = face.normal.normalized();
            int materialIndex = face.material_id;
            Tucano::Material::Mtl material = materials[materialIndex];

            const auto &specular = material.getSpecular();
            float EPSILON = 0.00001f;
            if (specular.x() > EPSILON || specular.y() > EPSILON || specular.z() > EPSILON) {

                // Reflection
                const auto hitFace = mesh.getFace(currentMaxDepthFaceId);
                const auto normal = hitFace.normal;
                Eigen::Vector3f reflection = direction - 2 * direction.dot(normal) * normal;
                const Eigen::Vector3f reflectionShading = traceRay(currentHitpoint, reflection, recursionDepth + 1);
                const Eigen::Vector3f weightedReflectionShading = {
                        reflectionShading.x() * specular.x(),
                        reflectionShading.y() * specular.y(),
                        reflectionShading.z() * specular.z()
                };

                // Refraction
                Eigen::Vector3f refraction = { 0, 0, 0 };
                const Eigen::Vector3f refractionShading = traceRay(currentHitpoint, refraction, recursionDepth + 1);
                const Eigen::Vector3f weightedRefractionShading = {
                        refractionShading.x() * specular.x(),
                        refractionShading.y() * specular.y(),
                        refractionShading.z() * specular.z()
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

        // If no intersection, grayish
        return {
        0.5,
        0.5,
        0.5
        };
    }
}

void Flyscene::traceFromY(int startY, int amountY, Eigen::Vector3f &origin, vector<vector<Eigen::Vector3f>> &pixel_data,
                          Eigen::Vector2i &image_size) {

    // for every pixel shoot a ray from the origin through the pixel coords
    for (int y = startY; y < startY + amountY; ++y) {
        for (int x = 0; x < image_size[0]; ++x) {
            // create a ray from the camera passing through the pixel (i,j)
            Eigen::Vector3f screen_coords = flycamera.screenToWorld(Eigen::Vector2f(x, y));

            // launch raytracing for the given ray and write result to pixel data
            pixel_data[x][y] = traceRay(origin, screen_coords, 0);
        }
    }
}
