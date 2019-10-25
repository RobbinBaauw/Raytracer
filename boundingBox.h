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

static int nodeCount = 0;
static int leafCount = 0;

static int deleteCount = 0;

#pragma once
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
	int baseCase = 3000;

public:
	bool hitByRay = false;

	boundingBox(void) = default;

	/**
	 * @brief Constructor of the boundingBox
	 * @param smallest corner
	 * @param biggest corner
	 */
	boundingBox(Eigen::Vector3f vmin, Eigen::Vector3f vmax) {
		this->vmin = vmin;
		this->vmax = vmax;
	}

	/**
	 * @brief Adds a faceIndex to the list of indices
	 * @param index of the face to be added
	 */
	void addFaceIndex(int faceIndex) {
		faceIndices.push_back(faceIndex);
	}

	/**
	 * @brief Stores the children boxes of this node.
	 * @param box on the smaller side
	 * @param box on the bigger side
	 */
	void setChildren(boundingBox lowerBox, boundingBox upperBox) {
		children = { lowerBox, upperBox };
	}

	void setVminIndex(float min, int index) {
		vmin(index) = min;
	}

	void setVmaxIndex(float max, int index) {
		vmax(index) = max;
	}

	//Return the list of faceIndices of the faces contained by the boundingBox
	std::vector<int> getFaceIndices() {
		return faceIndices;
	}

	Eigen::Vector3f getVmin() {
		return vmin;
	}

	Eigen::Vector3f getVmax() {
		return vmax;
	}

	/**
	 * @brief The function that splits the box in two on the average of the biggest side
	 * @param The used mesh as a reference "std::ref(mesh)"
	 */
	void splitBox(Tucano::Mesh& mesh) {
		++nodeCount;
		//This will be a recursive function so we will need a basecase, the minimum amount of faces alowed in a box
		if (faceIndices.size() < baseCase) {
			++leafCount;
			return;
		}
		//Get the index of the longest side of the box
		std::vector<float> side = { vmax(0) - vmin(0), vmax(1) - vmin(1) ,vmax(2) - vmin(2) };
		int sideIndex = std::max_element(side.begin(), side.end()) - side.begin();

		//Calculate the average point inside the box
		float sum = 0;
		float weight = 0;
		for (std::vector<int>::iterator it = faceIndices.begin(); it != faceIndices.end(); ++it) {
			Tucano::Face face = mesh.getFace(*it);
			sum += mesh.getVertex(face.vertex_ids[0])(sideIndex) +
				mesh.getVertex(face.vertex_ids[1])(sideIndex) +
				mesh.getVertex(face.vertex_ids[2])(sideIndex);
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

		for (std::vector<int>::iterator it = faceIndices.begin(); it != faceIndices.end(); ++it) {
			Tucano::Face face = mesh.getFace(*it);
			float first = mesh.getVertex(face.vertex_ids[0])(sideIndex);
			float sec = mesh.getVertex(face.vertex_ids[1])(sideIndex);
			float third = mesh.getVertex(face.vertex_ids[2])(sideIndex);

			if (first > avg && sec > avg && third > avg) {
				upperBox.addFaceIndex(*it);
			}
			else if (first < avg && sec < avg && third < avg) {
				lowerBox.addFaceIndex(*it);
			}
			else {
				lowerMax = max(first, max(sec, max(third, avg)));
				upperMin = min(first, min(sec, min(third, avg)));

				upperBox.addFaceIndex(*it);
				lowerBox.addFaceIndex(*it);
			}
		}

		upperBox.setVminIndex(upperMin, sideIndex);
		lowerBox.setVmaxIndex(lowerMax, sideIndex);

		//Perform recursive splitting of the boxes but only if the split actually did something.
		if (lowerBox.faceIndices.size() < 0.8 * faceIndices.size() && upperBox.faceIndices.size() < 0.8 * faceIndices.size()) {
			lowerBox.splitBox(std::ref(mesh));
			upperBox.splitBox(std::ref(mesh));
			setChildren(lowerBox, upperBox);
		}
		else {
			++leafCount;
		}
	}

	std::vector<boundingBox> getChildren() {
		return children;
	}

	/** @brief Returns a certain corner of the box.
	* 0 smallest x-y-z
	* 1 smallest y-z		biggest x
	* 2 smallest y			biggest x-z
	* 3 smallest x-y		biggest z
	* 4 smallest x			biggest y-z
	* 5 smallest x-z		biggest y
	* 6 smallest z			biggest x-y
	* 7 biggest x-y-z
	*/
	Eigen::Vector3f getCorner(int index) {
		// smallest x-y-z
		if (index == 0) return vmin;

		// smallest y-z		biggest x
		if (index == 1) return Eigen::Vector3f(vmax[0], vmin[1], vmin[2]);

		// smallest y		biggest x-z
		if (index == 2) return Eigen::Vector3f(vmax[0], vmin[1], vmax[2]);

		// smallest x-y		biggest z
		if (index == 3) return Eigen::Vector3f(vmin[0], vmin[1], vmax[2]);

		// smallest x		biggest y-z
		if (index == 4) return Eigen::Vector3f(vmin[0], vmax[1], vmax[2]);

		// smallest x-z		biggest y
		if (index == 5) return Eigen::Vector3f(vmin[0], vmax[1], vmin[2]);

		// smallest z		biggest x-y
		if (index == 6) return Eigen::Vector3f(vmax[0], vmin[1], vmax[2]);

		// biggest x-y-z
		//if (index == 7)
		return vmax;
	}

	/*
	 * Returns the size of the x-y-z axis of the cube.
	 * @param shapeModelMatrix the modelMatrix of the mesh, to transform the boundingBox to the actual size
	 */
	Eigen::Vector3f getShape(Eigen::Affine3f shapeModelMatrix) {
		Eigen::Vector3f minLoc = shapeModelMatrix * vmin;
		Eigen::Vector3f maxLoc = shapeModelMatrix * vmax;
		return {
			maxLoc[0] - minLoc[0],
			maxLoc[1] - minLoc[1],
			maxLoc[2] - minLoc[2]
		};
	}

	/*
	 * Renders the root box + starts recursively rendering till a certain depth
	 * @param flycamera, reference
	 * @param shapeModelMatrix, the modelmatrix of the mesh, to translate the cube to the center of the mesh
	 * @param onlyIntersected if set to true we only show the leaves that are hit by the debug ray
	 */
	void renderLeafBoxes(Tucano::Flycamera& flycamera, Tucano::Camera& scene_light, Eigen::Affine3f shapeModelMatrix, bool onlyIntersected) {
		if (!children.empty()) {
			children[0].renderLeafBoxes(std::ref(flycamera), std::ref(scene_light), shapeModelMatrix, onlyIntersected);
			children[1].renderLeafBoxes(std::ref(flycamera), std::ref(scene_light), shapeModelMatrix, onlyIntersected);
		}
		else {
			//Render when we don't want only intersected, and if we do only want intersected then should be hit by the ray as well.
			bool perform = !onlyIntersected || (onlyIntersected && hitByRay);
			if (perform) {
				Eigen::Vector3f shape = getShape(shapeModelMatrix);
				Tucano::Shapes::Box bounding = Tucano::Shapes::Box(shape[0], shape[1], shape[2]);
				bounding.resetModelMatrix();
				bounding.modelMatrix()->translate(shapeModelMatrix * ((vmax + vmin) / 2));
				float r = ((double)_CSTDLIB_::rand() / (RAND_MAX));
				float g = ((double)_CSTDLIB_::rand() / (RAND_MAX));
				float b = ((double)_CSTDLIB_::rand() / (RAND_MAX));
				bounding.setColor(Eigen::Vector4f(r, g, b, 0.1));
				bounding.render(flycamera, scene_light);

				if (deleteCount < 1) {
					std::cout << "vmin" << shapeModelMatrix * vmin << std::endl;
					std::cout << "vmax" << shapeModelMatrix * vmax << std::endl;

					std::cout << "Shape " << getShape(shapeModelMatrix) << std::endl;
					std::cout << "modelmatrix" << bounding.getModelMatrix().matrix() << std::endl;
					std::cout << "shapematrix" << bounding.getShapeMatrix().matrix() << std::endl;
					std::cout << "shapemodelmatrix" << bounding.getShapeModelMatrix().matrix() << std::endl;
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
	bool boxIntersection(Eigen::Vector3f origin, Eigen::Vector3f dest) {
		Eigen::Vector3f dir = (dest - origin).normalized();
		float tx_min = (vmin.x() - origin.x()) / dir.x();
		float tx_max = (vmax.x() - origin.x()) / dir.x();

		if (tx_min > tx_max) swap(tx_min, tx_max);

		float ty_min = (vmin.y() - origin.y()) / dir.y();
		float ty_max = (vmax.y() - origin.y()) / dir.y();

		if (ty_min > ty_max) swap(ty_min, ty_max);

		float tz_min = (vmin.z() - origin.z()) / dir.z();
		float tz_max = (vmax.z() - origin.z()) / dir.z();

		if (tz_min > tz_max) swap(tz_min, tz_max);

		float tin_x = min(tx_min, tx_max);
		float tout_x = max(tx_min, tx_max);

		float tin_y = min(ty_min, ty_max);
		float tout_y = max(ty_min, ty_max);

		float tin_z = min(tz_min, tz_max);
		float tout_z = max(tz_min, tz_max);

		float tin = max(tin_x, max(tin_y, tin_z));
		float tout = min(tout_x, min(tout_y, tout_z));

		if (tin > tout || tout < 0) {
			return false;
		}
		return true;
	}

	/*
	* Returns the unique face indices of the boxes that intersect with the light ray.
	* @param box, the bounding box
	* @param origin, the origin of the ray
	* @param dest, the destination
	* @param intersectingFaces, an empty vector list of face indices.
	*/
	void intersectingBoxes(Eigen::Vector3f origin, Eigen::Vector3f dest, std::vector<int> intersectingFaces) {
		if (getChildren().size() == 0) {
			if (boxIntersection(origin, dest)) {
				hitByRay = true;
				for (int k = 0; k < getFaceIndices().size(); ++k) {
					//intersectingFaces.push_back(getFaceIndices().at(k));
				}
			}
			else hitByRay = false;
			return;
		}


		children[0].intersectingBoxes(origin, dest, intersectingFaces);
		children[1].intersectingBoxes(origin, dest, intersectingFaces);
	}
};


