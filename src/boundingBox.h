#ifndef __BOUNDINGBOX__
#define __BOUNDINGBOX__

// Must be included before glfw.
#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <tucano/effects/phongmaterialshader.hpp>
#include <tucano/mesh.hpp>
#include <tucano/shapes/camerarep.hpp>
#include <tucano/shapes/cylinder.hpp>
#include <tucano/shapes/sphere.hpp>
#include <tucano/utils/flycamera.hpp>
#include <tucano/utils/imageIO.hpp>
#include <tucano/utils/mtlIO.hpp>
#include <tucano/utils/objimporter.hpp>
#include "defs.h"

static int nodeCount = 0;
static int leafCount = 0;

static int deleteCount = 0;

class boundingBox {
private:
    //The indices of all the faces contained in this boundingBox
    std::vector<int> faceIndices;

    //The minimum corner of the boundingBox
    Eigen::Vector3f vmin;

    //The maximum corner of the boundingBox
    Eigen::Vector3f vmax;

    //The children of the boundingBox, either 0 or 2. Formed by splitting the box in two along its biggest side
    std::vector<boundingBox> children;

    //If we have this amount of faces in a box it shouldn't be split any further
    int baseCase = 50;

public:
    bool hitByRay = false;

    boundingBox() = default;

    /**
     * @brief Constructor of the boundingBox
     * @param smallest corner
     * @param biggest corner
     */
    boundingBox(Eigen::Vector3f& vmin, Eigen::Vector3f& vmax) : vmin(vmin), vmax(vmax) {
    }

    /**
     * @brief Adds a faceIndex to the list of indices
     * @param index of the face to be added
     */
    void addFaceIndex(int faceIndex) {
        faceIndices.push_back(faceIndex);
    }

    std::vector<boundingBox> getChildren() {
        return children;
    }

    /**
     * @brief Stores the children boxes of this node.
     * @param box on the smaller side
     * @param box on the bigger side
     */
    void setChildren(boundingBox& lowerBox, boundingBox& upperBox) {
        children = {
                lowerBox, upperBox
        };
    }

    void setVminIndex(float min, int index) {
        vmin(index) = min;
    }

    void setVmaxIndex(float max, int index) {
        vmax(index) = max;
    }

    /**
     * @brief The function that splits the box in two on the average of the biggest side
     * @param The used mesh as a reference "std::ref(mesh)"
     */
    void splitBox(PrecomputedData& precomputedData) {
        ++nodeCount;

        //This will be a recursive function so we will need a basecase, the minimum amount of faces alowed in a box
        if (faceIndices.size() < baseCase) {
            ++leafCount;
            return;
        }
        //Get the index of the longest side of the box
        std::vector<float> side = {vmax(0) - vmin(0), vmax(1) - vmin(1), vmax(2) - vmin(2)};
        int sideIndex = std::max_element(side.begin(), side.end()) - side.begin();

        //Calculate the average point inside the box
        float sum = 0;
        float weight = 0;
        for (auto &it : faceIndices) {
            const auto vertexIds = precomputedData.faceVertexIds[it];
            sum += precomputedData.vertices[get<0>(vertexIds)](sideIndex) +
                    precomputedData.vertices[get<1>(vertexIds)](sideIndex) +
                    precomputedData.vertices[get<2>(vertexIds)](sideIndex);

            weight += 3;
        }
        float avg = sum / weight;
        /*std::cout << "Avg: " << avg << std::endl;
        std::cout << "Longst side " << sideIndex << std::endl;*/

        //Create the new upper corner for the lower boundingBox
        Eigen::Vector3f lowerVmax = vmax;
        //Setting avg for now will change
        lowerVmax(sideIndex) = avg;

        //Create the new lower corner for the upper boundinBox
        Eigen::Vector3f upperVmin = vmin;
        //Setting avg for now will change later in scope
        upperVmin(sideIndex) = avg;

        boundingBox lowerBox = boundingBox(vmin, lowerVmax);
        boundingBox upperBox = boundingBox(upperVmin, vmax);

        float lowerMax = std::numeric_limits<float>::min();
        float upperMin = std::numeric_limits<float>::max();

        for (auto &it : faceIndices) {
            const auto vertexIds = precomputedData.faceVertexIds[it];

            float first = precomputedData.vertices[get<0>(vertexIds)](sideIndex);
            float sec = precomputedData.vertices[get<1>(vertexIds)](sideIndex);
            float third = precomputedData.vertices[get<2>(vertexIds)](sideIndex);

            if (first > avg && sec > avg && third > avg) {
                upperBox.addFaceIndex(it);
            } else if (first < avg && sec < avg && third < avg) {
                lowerBox.addFaceIndex(it);
            } else {
                lowerMax = max(first, max(sec, max(third, avg)));
                upperMin = min(first, min(sec, min(third, avg)));

                upperBox.addFaceIndex(it);
                lowerBox.addFaceIndex(it);
            }
        }

        upperBox.setVminIndex(upperMin, sideIndex);
        lowerBox.setVmaxIndex(lowerMax, sideIndex);

        //Perform recursive splitting of the boxes but only if the split actually did something.
        if (lowerBox.faceIndices.size() < 0.8 * faceIndices.size() && upperBox.faceIndices.size() < 0.8 * faceIndices.size()) {
            lowerBox.splitBox(precomputedData);
            upperBox.splitBox(precomputedData);
            setChildren(lowerBox, upperBox);
        } else {
            ++leafCount;
        }
    }

    /*
     * Returns the size of the x-y-z axis of the cube.
     * @param shapeModelMatrix the modelMatrix of the mesh, to transform the boundingBox to the actual size
     */
    Eigen::Vector3f getShape() {
        return {
                vmax[0] - vmin[0],
                vmax[1] - vmin[1],
                vmax[2] - vmin[2]
        };
    }

    /*
     * Renders the root box + starts recursively rendering till a certain depth
     * @param flycamera, reference
     * @param shapeModelMatrix, the modelmatrix of the mesh, to translate the cube to the center of the mesh
     * @param onlyIntersected if set to true we only show the leaves that are hit by the debug ray
     */
    void renderLeafBoxes(const Tucano::Flycamera &flycamera, const Tucano::Camera &scene_light, const bool onlyIntersected) {
        if (!children.empty()) {
            children[0].renderLeafBoxes(flycamera, scene_light, onlyIntersected);
            children[1].renderLeafBoxes(flycamera, scene_light, onlyIntersected);
        } else {
            //Render when we don't want only intersected, and if we do only want intersected then should be hit by the ray as well.
            bool perform = !onlyIntersected || (onlyIntersected && hitByRay);
            if (perform) {
                Eigen::Vector3f shape = getShape();
                Tucano::Shapes::Box bounding = Tucano::Shapes::Box(shape[0], shape[1], shape[2]);
                bounding.resetModelMatrix();
                bounding.modelMatrix()->translate(((vmax + vmin) / 2));
                auto r = (float) ((double) _CSTDLIB_::rand() / (RAND_MAX));
                auto g = (float) ((double) _CSTDLIB_::rand() / (RAND_MAX));
                auto b = (float) ((double) _CSTDLIB_::rand() / (RAND_MAX));
                bounding.setColor(Eigen::Vector4f(r, g, b, 0.1));
                bounding.render(flycamera, scene_light);

                if (deleteCount < 1) {
                    std::cout << "modelmatrix" << bounding.getModelMatrix().matrix() << std::endl;
                    std::cout << "shapematrix" << bounding.getShapeMatrix().matrix() << std::endl;
                    ++deleteCount;
                }
            }
        }
    }

    static int getLeaf() {
        return leafCount;
    }

    static int getNode() {
        return nodeCount;
    }

    /*
    * Returns true if there is intersection between the bounding box and ray.
    * @param box, this is the bounding box
    * @param origin, the origin of the ray
    * @param dest, the destination
    */
    bool boxIntersection(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction) {
        float tx_min = (vmin.x() - origin.x()) / direction.x();
        float tx_max = (vmax.x() - origin.x()) / direction.x();

        if (tx_min > tx_max) swap(tx_min, tx_max);

        float ty_min = (vmin.y() - origin.y()) / direction.y();
        float ty_max = (vmax.y() - origin.y()) / direction.y();

        if (ty_min > ty_max) swap(ty_min, ty_max);

        float tz_min = (vmin.z() - origin.z()) / direction.z();
        float tz_max = (vmax.z() - origin.z()) / direction.z();

        if (tz_min > tz_max) swap(tz_min, tz_max);

        float tin_x = min(tx_min, tx_max);
        float tout_x = max(tx_min, tx_max);

        float tin_y = min(ty_min, ty_max);
        float tout_y = max(ty_min, ty_max);

        float tin_z = min(tz_min, tz_max);
        float tout_z = max(tz_min, tz_max);

        float tin = max(tin_x, max(tin_y, tin_z));
        float tout = min(tout_x, min(tout_y, tout_z));

        return !(tin > tout || tout < 0);
    }

    /*
    * Returns the unique face indices of the boxes that intersect with the light ray.
    * @param box, the bounding box
    * @param origin, the origin of the ray
    * @param dest, the destination
    * @param intersectingFaces, an empty vector list of face indices.
    */
    void intersectingBoxes(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, std::vector<int> &intersectingFaces) {
        if (boxIntersection(origin, direction)) {
            if (children.empty()) {
                hitByRay = true;
                intersectingFaces.insert(intersectingFaces.end(), faceIndices.begin(), faceIndices.end());
            } else {
                children[0].intersectingBoxes(origin, direction, intersectingFaces);
                children[1].intersectingBoxes(origin, direction, intersectingFaces);
            }
        } else {
            hitByRay = false;
        }
    }
};

#endif // BOUNDINGBOX