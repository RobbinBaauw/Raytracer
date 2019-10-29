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

static int depth = 0;

class boundingBox {
private:
    //The indices of all the faces contained in this boundingBox
    std::vector<int> faceIndices;

    //The children of the boundingBox, either 0 or 2. Formed by splitting the box in two along its biggest side
    std::vector<boundingBox> children;

    //If we have this amount of faces in a box it shouldn't be split any further
    int baseCase = 10;

    Tucano::Shapes::Box visualization;

public:
    bool hitByRay = false;

    //The minimum corner of the boundingBox
    Eigen::Vector3f vmin;

    //The maximum corner of the boundingBox
    Eigen::Vector3f vmax;

    boundingBox() = default;

    /**
     * @brief Constructor of the boundingBox
     * @param smallest corner
     * @param biggest corner
     */
    boundingBox(Eigen::Vector3f &vmin, Eigen::Vector3f &vmax) : vmin(vmin), vmax(vmax) {
    }

    /**
     * @brief Adds a faceIndex to the list of indices
     * @param index of the face to be added
     */
    inline void addFaceIndex(int faceIndex) {
        faceIndices.emplace_back(faceIndex);
    }

    /**
     * @brief The function that splits the box in two on the average of the biggest side
     * @param The used mesh as a reference "std::ref(mesh)"
     */
    void splitBox(PrecomputedData &precomputedData) {
        ++nodeCount;

        //This will be a recursive function so we will need a basecase, the minimum amount of faces alowed in a box
        if (faceIndices.size() < baseCase) {
            ++leafCount;
            return;
        }
        //Get the index of the longest side of the box
        std::vector<float> side = {
                vmax(0) - vmin(0),
                vmax(1) - vmin(1),
                vmax(2) - vmin(2)
        };

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

        Eigen::Vector3f upperMin = vmax;
        Eigen::Vector3f upperMax = vmin;

        Eigen::Vector3f lowerMin = vmax;
        Eigen::Vector3f lowerMax = vmin;

        unsigned int amountInRandomBox = 0;

        for (auto &it : faceIndices) {
            const auto vertexIds = precomputedData.faceVertexIds[it];

            const Eigen::Vector3f &v0 = precomputedData.vertices[get<0>(vertexIds)];
            const Eigen::Vector3f &v1 = precomputedData.vertices[get<1>(vertexIds)];
            const Eigen::Vector3f &v2 = precomputedData.vertices[get<2>(vertexIds)];

            float first = v0(sideIndex);
            float sec = v1(sideIndex);
            float third = v2(sideIndex);

            const auto isInUpperBox = first > avg && sec > avg && third > avg;
            const auto isInLowerBox = first < avg && sec < avg && third < avg;

            if (isInUpperBox || (!isInLowerBox && amountInRandomBox % 2 == 0)) {
                upperBox.addFaceIndex(it);

                upperMin = {
                        min(v0.x(), min(v1.x(), min(v2.x(), upperMin.x()))),
                        min(v0.y(), min(v1.y(), min(v2.y(), upperMin.y()))),
                        min(v0.z(), min(v1.z(), min(v2.z(), upperMin.z())))
                };

                upperMax = {
                        max(v0.x(), max(v1.x(), max(v2.x(), upperMax.x()))),
                        max(v0.y(), max(v1.y(), max(v2.y(), upperMax.y()))),
                        max(v0.z(), max(v1.z(), max(v2.z(), upperMax.z())))
                };

                if (!isInUpperBox) {
                    amountInRandomBox++;
                }

            } else {
                lowerBox.addFaceIndex(it);

                lowerMin = {
                        min(v0.x(), min(v1.x(), min(v2.x(), lowerMin.x()))),
                        min(v0.y(), min(v1.y(), min(v2.y(), lowerMin.y()))),
                        min(v0.z(), min(v1.z(), min(v2.z(), lowerMin.z())))
                };

                lowerMax = {
                        max(v0.x(), max(v1.x(), max(v2.x(), lowerMax.x()))),
                        max(v0.y(), max(v1.y(), max(v2.y(), lowerMax.y()))),
                        max(v0.z(), max(v1.z(), max(v2.z(), lowerMax.z())))
                };

                if (!isInLowerBox) {
                    amountInRandomBox++;
                }
            }
        }

        upperBox.vmin = upperMin;
        upperBox.vmax = upperMax;

        lowerBox.vmin = lowerMin;
        lowerBox.vmax = lowerMax;

        //Perform recursive splitting of the boxes but only if the split actually did something.
        if (lowerBox.faceIndices.size() < 0.8 * faceIndices.size() && upperBox.faceIndices.size() < 0.8 * faceIndices.size()) {
            lowerBox.splitBox(precomputedData);
            upperBox.splitBox(precomputedData);
            children = {
                    lowerBox, upperBox
            };
        } else {
            ++leafCount;
        }

        Eigen::Vector3f shape = getShape();
        visualization = Tucano::Shapes::Box(shape[0], shape[1], shape[2]);
        visualization.resetModelMatrix();
        visualization.modelMatrix()->translate(((vmax + vmin) / 2));
        auto r = (float) ((double) _CSTDLIB_::rand() / (RAND_MAX));
        auto g = (float) ((double) _CSTDLIB_::rand() / (RAND_MAX));
        auto b = (float) ((double) _CSTDLIB_::rand() / (RAND_MAX));
        visualization.setColor(Eigen::Vector4f(r, g, b, 0.1));
    }

    void computeDepth() {
        depth = computeDepth(0);
    }

    int computeDepth(int currDepth) {
        if (!children.empty()) {
            return max(children[0].computeDepth(currDepth + 1), children[1].computeDepth(currDepth + 1));
        } else {
            return currDepth;
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
    void renderLeafBoxes(const Tucano::Flycamera &flycamera, const Tucano::Camera &scene_light, const bool onlyIntersected, int &requiredSplitDepth, const int currentSplitDepth) {
        if (requiredSplitDepth == -1) {
            return;
        } else if (requiredSplitDepth > depth) {
            requiredSplitDepth = -1;
        } else if (requiredSplitDepth == currentSplitDepth) {
            //Render when we don't want only intersected, and if we do only want intersected then should be hit by the ray as well.
            bool perform = !onlyIntersected || (onlyIntersected && hitByRay);
            if (perform) {
                visualization.render(flycamera, scene_light);
            }
        } else if (!children.empty()) {
            children[0].renderLeafBoxes(flycamera, scene_light, onlyIntersected, requiredSplitDepth, currentSplitDepth + 1);
            children[1].renderLeafBoxes(flycamera, scene_light, onlyIntersected, requiredSplitDepth, currentSplitDepth + 1);
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
    void intersectingBoxes(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, std::vector<std::vector<int> *> &intersectingFaces) {
        if (boxIntersection(origin, direction)) {
			hitByRay = true;
            if (children.empty()) {
                intersectingFaces.emplace_back(&faceIndices);
            } else {
                children[0].intersectingBoxes(origin, direction, intersectingFaces);
                children[1].intersectingBoxes(origin, direction, intersectingFaces);
            }
        } else {
            hitByRay = false;
        }
    }

	void resetHitByRay() {
		hitByRay = false;
		if (!children.empty()) {
			children[0].resetHitByRay();
			children[1].resetHitByRay();
		}
	}
};

#endif // BOUNDINGBOX