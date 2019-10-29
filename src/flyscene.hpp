#ifndef __FLYSCENE__
#define __FLYSCENE__

// Must be included before glfw.
#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <tucano/effects/phongmaterialshader.hpp>
#include <tucano/mesh.hpp>
#include <tucano/shapes/camerarep.hpp>
#include <tucano/shapes/cylinder.hpp>
#include <tucano/shapes/sphere.hpp>
#include <tucano/shapes/box.hpp>
#include <tucano/utils/flycamera.hpp>
#include <tucano/utils/imageIO.hpp>
#include <tucano/utils/mtlIO.hpp>
#include <tucano/utils/objimporter.hpp>
#include "boundingBox.h"

// Global variables
#define HARDSHADOW // Enables hardshadow
#define SOFTSHADOW // Enables softshadow

#define INFOTIMESTAMPING // Logs info timestamping
//#define DEBUGTIMESTAMPING // Logs debug timestamping

#define INFO // Logs info stuff
//#define DEBUG // Logs debug stuff

//#define PAINTGL // Enables preview window

#define SOFTSHADOWRADIUS 0.3f // The radius of the lights for soft shadows
#define MAXSOFTSHADOWPOINTS 12 // The amount of points used for soft shadows

#define SSAALEVEL 3 // The SSAA level (3 => 3^2 = 9 times as many pixels)

//#define THREADCOUNTOVERRIDE 11 // The amount of threads (if not defined it uses max)

#define MAXREFLECTIONS 5 // The maximum amount of reflections / refractions
#define MAXDEBUGREFLECTIONS 10 // The maximum amount of reflections in the debug tracer

class Flyscene {

public:
    Flyscene() = default;

    /**
     * @brief Initializes the shader effect
     * @param width Window width in pixels
     * @param height Window height in pixels
     */
    void initialize(int width, int height);

    /**
     * Repaints screen buffer.
     **/
    virtual void paintGL();

    /**
     * Perform a single simulation step.
     **/
    virtual void simulate(GLFWwindow *window);

    /**
     * Returns the pointer to the flycamera instance
     * @return pointer to flycamera
     **/
    Tucano::Flycamera *getCamera() { return &flycamera; }

    /**
     * @brief Add a new light source
     */
    void addLight() { lights.push_back(flycamera.getCenter()); }

    /**
     * @brief Create a debug ray at the current camera location and passing
     * through pixel that mouse is over
     * @param mouse_pos Mouse cursor position in pixels
     */
    void createDebugRay(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, int recursionDepth);

    void startDebugRay(const Eigen::Vector2f &mouseCoords);

    /**
     * @brief raytrace your scene from current camera position
     */
    void raytraceScene();

    /**
    * @Brief function that calculates basic shading of an intersected face
    */
    Eigen::Vector3f shadeOffFace(int faceIndex, const Eigen::Vector3f &rayDirection, const Eigen::Vector3f &hitPosition, const Eigen::Vector3f &lightIntensity);

    std::vector<Eigen::Vector3f> boundingVectors();

    /**
     * @brief trace a single ray from the camera passing through dest
     * @param origin Ray origin
     * @param dest Other point on the ray, usually screen coordinates
     * @return a RGB color
     */
    Eigen::Vector3f traceRay(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, int recursionDepth);

    bool intersects(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                    int &faceId, Eigen::Vector3f &hitpoint,
                    Eigen::Vector3f &reflection, Eigen::Vector3f &refraction);

    bool triangleIntersection(float &currentMaxDepth, const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                              int &faceId, Eigen::Vector3f &hitpoint,
                              Eigen::Vector3f &reflection, Eigen::Vector3f &refraction
    );

    bool sphereIntersection(float &currentMaxDepth, const Eigen::Vector3f &origin, const Eigen::Vector3f &direction,
                            int &faceId, Eigen::Vector3f &hitpoint,
                            Eigen::Vector3f &reflection, Eigen::Vector3f &refraction
    );

    void tracePixels(unsigned int threadId,
                     unsigned int threads,
                     const Eigen::Vector3f &origin,
                     vector<vector<Eigen::Vector3f>> &pixel_data,
                     int xSize,
                     int ySize);

    void precomputeData();

    void precomputeLights();

    /**
	* @brief function which calculates soft shadows.
	*/
    Eigen::Vector3f getLightIntensity(const Eigen::Vector3f &hitPosition);

    bool renderIntersection = false;
    int splitPreviewDepth = -1;
	int debugReflectionDepth = -1;

private:

    PrecomputedData precomputedData;

    // A simple phong shader for rendering meshes
    Tucano::Effects::PhongMaterial phong;

  // A fly through camera
  Tucano::Flycamera flycamera;

    // A camera representation for animating path (false means that we do not
    // render front face)
    Tucano::Shapes::CameraRep camerarep = Tucano::Shapes::CameraRep(false);

  // a frustum to represent the camera in the scene
  Tucano::Shapes::Sphere lightrep;

  // light sources for ray tracing
  vector<Eigen::Vector3f> lights;

    // sheres for ray tracing
    vector<Sphere> spheres;

    // Scene light represented as a camera
    Tucano::Camera scene_light;

    /// A very thin cylinder to draw a debug ray
    Tucano::Shapes::Cylinder ray = Tucano::Shapes::Cylinder(0.1, 1.0, 16, 64);

    // Scene meshes
    Tucano::Mesh mesh;

    /// MTL materials
    vector<Tucano::Material::Mtl> materials;

    // boundingBox that contains the whole mesh and is the root
    boundingBox boxMain;
};

#endif // FLYSCENE
