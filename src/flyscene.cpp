#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include <cassert>
#include <thread>
#include <chrono>
#include "../boundingBox.h"

const int MAXRECURSION = 5;
const int MAXDEBUGRECURSION = 10;

boundingBox boxMain;
bool renderBoxBool = false;

void Flyscene::initialize(int width, int height) {
    // initiliaze the Phong Shading effect for the Opengl Previewer
    phong.initialize();

    // set the camera's projection matrix
    flycamera.setPerspectiveMatrix(60.0, (float) width / (float) height, 0.1f, 100.0f);
    flycamera.setViewport(Eigen::Vector2f((float) width, (float) height));

    // load the OBJ file and materials
    //Tucano::MeshImporter::loadObjFile(mesh, materials,"resources/models/dodgeColorTest.obj");
    Tucano::MeshImporter::loadObjFile(mesh, materials, "resources/models/bunny.obj");

  // set the camera's projection matrix
  flycamera.setPerspectiveMatrix(60.0, width / (float)height, 0.1f, 100.0f);
  flycamera.setViewport(Eigen::Vector2f((float)width, (float)height));

  // load the OBJ file and materials
  Tucano::MeshImporter::loadObjFile(mesh, materials,
                                    "resources/models/plane.obj");


  mesh.normalizeModelMatrix();

    startDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

    camerarep.resetModelMatrix();
    camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());

    // the debug ray is a cylinder, set the radius and length of the cylinder
    ray.setSize(0.005, 10.0);

	std::vector<Eigen::Vector3f> boundaries = boundingVectors();
	boxMain = boundingBox(boundaries[0], boundaries[1]);

	int numb_faces = mesh.getNumberOfFaces();
	for (int i = 0; i < numb_faces; ++i) {
		boxMain.addFaceIndex(i);
	}

	boxMain.splitBox(std::ref(mesh));
	std::cout << "Node count is: " << boundingBox::getNode() << std::endl;
	std::cout << "Leaf count is: " << boundingBox::getLeaf() << std::endl;
	
	std::vector<boundingBox> currChildren = boxMain.getChildren();
	int depth = 0;
	while (currChildren.size() > 0) {
		++depth;
		currChildren = currChildren[0].getChildren();
	}
	std::cout << "Depth of the lefmost branch is " << depth << std::endl;

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


	if (renderBoxBool) {
		boxMain.renderBox(std::ref(flycamera), std::ref(scene_light), mesh.getShapeModelMatrix(), 0);
	}

	/*Eigen::Vector3f shape = boxMain.getShape(mesh.getShapeModelMatrix());
	Tucano::Shapes::Box bounding = Tucano::Shapes::Box(shape[0], shape[1], shape[2]);
	bounding.resetModelMatrix();
	bounding.modelMatrix()->translate(mesh.getShapeModelMatrix() * ((boxMain.getVmax() + boxMain.getVmin())/2));
	bounding.setColor(Eigen::Vector4f(0.1, 1, 0.4, 0.1));
	bounding.renderLines();
	bounding.render(flycamera, scene_light);*/

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

    float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 0.2f : 0.0f) -
               (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 0.2f : 0.0f);

    float dy = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS ? 0.2f : 0.0f) -
               (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ? 0.2f : 0.0f);

    float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 0.2f : 0.0f) -
               (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 0.2f : 0.0f);

	//This code only shows the leaf boundingBoxes when B is pressed.
	renderBoxBool = false;
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS) renderBoxBool = true;

    flycamera.translate(dx, dy, dz);
}

void Flyscene::startDebugRay(const Eigen::Vector2f& mouseCoords) {

    rays.clear();

    // from pixel position to world coordinates
    Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouseCoords);

  // render the ray and camera representation for ray debug
  ray.render(flycamera, scene_light);
  camerarep.render(flycamera, scene_light);

    createDebugRay(flycamera.getCenter(), dir, 0);
}

void Flyscene::createDebugRay(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, int recursionDepth) {

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

    if(doesIntersect(origin, direction, faceId, hitPoint, reflection, refraction) && recursionDepth < MAXDEBUGRECURSION) {

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

    const auto threadCount = thread::hardware_concurrency();
//    const auto threadCount = 1;
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

Eigen::Vector3f Flyscene::shadeOffFace(int faceIndex, const Eigen::Vector3f& origin, const Eigen::Vector3f& hitPosition) {

    Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);

    Tucano::Face face = mesh.getFace(faceIndex);

    int materialIndex = face.material_id;
    if (materialIndex == -1) {
        return color;
    }

    Tucano::Material::Mtl material = materials[materialIndex];

    Eigen::Vector3f lightIntensity = Eigen::Vector3f(1, 1, 1);

    // Interpolating the normal
    const auto currVertexIds = face.vertex_ids;

    // TODO add drawing of why this way of calculating is logical
    const Eigen::Vector3f v0 = (mesh.getShapeModelMatrix() * mesh.getVertex(currVertexIds[0])).head(3);
    const Eigen::Vector3f v1 = (mesh.getShapeModelMatrix() * mesh.getVertex(currVertexIds[1])).head(3);
    const Eigen::Vector3f v2 = (mesh.getShapeModelMatrix() * mesh.getVertex(currVertexIds[2])).head(3);

    const auto v0Normal = mesh.getNormal(currVertexIds[0]).normalized();
    const auto v1Normal = mesh.getNormal(currVertexIds[1]).normalized();
    const auto v2Normal = mesh.getNormal(currVertexIds[2]).normalized();

    const auto areaV1V2Hitpoint = (v1 - hitPosition).cross(v2 - hitPosition).norm() * 0.5;
    const auto areaV0V2Hitpoint = (v0 - hitPosition).cross(v2 - hitPosition).norm() * 0.5;
    const auto areaV0V1Hitpoint = (v0 - hitPosition).cross(v1 - hitPosition).norm() * 0.5;

    Eigen::Vector3f faceNormal = areaV1V2Hitpoint * v0Normal + areaV0V2Hitpoint * v1Normal + areaV0V1Hitpoint * v2Normal;
    faceNormal = faceNormal / faceArea;
    faceNormal.normalize();

    // Iterate over all the present lights
    for (const Eigen::Vector3f& lightPosition : lights) {

        Eigen::Vector3f lightDirection = (lightPosition - hitPosition).normalized();

        // Ambient term
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
        Eigen::Vector3f minSum = colorSum.cwiseMax(0.0).cwiseMin(1.0);
        minSum[3] = material.getOpticalDensity();

        color += minSum;
    }

    return color;
}

Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f &origin, Eigen::Vector3f &dest, int recursionDepth) {

    Eigen::Vector3f hitPoint;
    int faceId;

    Eigen::Vector3f reflection;
    Eigen::Vector3f refraction;

Eigen::Vector3f Flyscene::traceRay(const Eigen::Vector3f &origin, const Eigen::Vector3f &dest, const int recursionDepth) {

    // Get direction of ray
    const auto direction = (dest - origin).normalized();

    if (doesIntersect(origin, direction, faceId, hitPoint, reflection, refraction)) {
        const Eigen::Vector3f localShading = shadeOffFace(faceId, origin, hitPoint);

        if (recursionDepth < MAXRECURSION) {
            Tucano::Face face = mesh.getFace(faceId);
            Eigen::Vector3f faceNormal = face.normal.normalized();

            int materialIndex = face.material_id;
            if (materialIndex == -1) {
                return localShading;
            }

            Tucano::Material::Mtl material = materials[materialIndex];

            const auto &specular = material.getSpecular();
            float EPSILON = 0.00001f;
            if (specular.x() > EPSILON || specular.y() > EPSILON || specular.z() > EPSILON) {

                // Reflection
                reflection.normalize();
                Eigen::Vector3f reflectionDirection = hitPoint + reflection;

                const Eigen::Vector3f reflectionShading = traceRay(hitPoint, reflectionDirection, recursionDepth + 1);
                const Eigen::Vector3f weightedReflectionShading = {
                reflectionShading.x() * specular.x(),
                reflectionShading.y() * specular.y(),
                reflectionShading.z() * specular.z()
                };

                // Refraction
                refraction.normalize();
                Eigen::Vector3f refractionDirection = hitPoint + refraction;

                const Eigen::Vector3f refractionShading = traceRay(hitPoint, refractionDirection, recursionDepth + 1);
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

bool Flyscene::doesIntersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                             int &faceId, Eigen::Vector3f &hitpoint,
                             Eigen::Vector3f &reflection, Eigen::Vector3f &refraction) {

    bool hasIntersected = false;
    float currentMaxDepth = numeric_limits<float>::max();

    auto nrOfFaces = mesh.getNumberOfFaces();
    for (int i = 0; i < nrOfFaces; i++) {
        // Retrieve the current face and its vertex ids
        const auto currFace = mesh.getFace(i);
        const auto currVertexIds = currFace.vertex_ids;

        assert(currVertexIds.size() == 3);

        // Create the vertices
        const auto v0 = (mesh.getShapeModelMatrix() * mesh.getVertex(currVertexIds[0])).head(3);
        const auto v1 = (mesh.getShapeModelMatrix() * mesh.getVertex(currVertexIds[1])).head(3);
        const auto v2 = (mesh.getShapeModelMatrix() * mesh.getVertex(currVertexIds[2])).head(3);

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

            reflection = direction - 2 * direction.dot(normal) * normal;
            refraction = Eigen::Vector3f(0, 0, 0); // TODO

            hitpoint = hitPoint;
            faceId = i;
        }
    }

    return hasIntersected;
}

void Flyscene::traceFromY(int startY, int amountY, Eigen::Vector3f &origin, vector<vector<Eigen::Vector3f>> &pixel_data,
                          Eigen::Vector2i &image_size) {

    // for every pixel shoot a ray from the origin through the pixel coords
    for (int y = startY; y < startY + amountY; ++y) {
        for (int x = 0; x < image_size[0]; ++x) {
            // create a ray from the camera passing through the pixel (i,j)
            Eigen::Vector3f screen_coords = flycamera.screenToWorld(Eigen::Vector2f(x, y));

            // launch raytracing for the given ray and write result to pixel data
            const Eigen::Vector3f &colorOut = traceRay(origin, screen_coords, 0);

            pixel_data[y][x] = colorOut;
        }
    }
}

std::vector<Eigen::Vector3f> Flyscene::boundingVectors() {
	int num_verts = mesh.getNumberOfVertices();
	Eigen::Vector3f vmin = { mesh.getVertex(0).x(), mesh.getVertex(0).y(), mesh.getVertex(0).z()};
	Eigen::Vector3f vmax = { mesh.getVertex(0).x(), mesh.getVertex(0).y(), mesh.getVertex(0).z() };
	for (int i = 0; i < num_verts; ++i) {
		Eigen::Vector4f vertex = mesh.getVertex(i);
		vmin(0) = min(vmin.x(), vertex.x());
		vmax(0) = max(vmax.x(), vertex.x());

		vmin(1) = min(vmin.y(), vertex.y());
		vmax(1) = max(vmax.y(), vertex.y());

		vmin(2) = min(vmin.z(), vertex.z());
		vmax(2) = max(vmax.z(), vertex.z());
	}
	return { vmin, vmax };
}
