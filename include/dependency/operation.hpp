/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 15:14:31
 * @LastEditTime: 2022-06-21 15:30:28
 * @LastEditors: ChenRP07
 * @Description: Header and Implement of some opeartions.
 */
#ifndef _LIB_OPERATION_H_
#define _LIB_OPERATION_H_
#include <Eigen/Dense>
#include <cfloat>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace vvs {
namespace operation {

	/***
	 * @description: __result.geometry = __a.geometry + __b.geometry
	 * @param {const PointTA&} __a
	 * @param {const PointTB&} __b
	 * @return {__result}
	 */
	template <typename PointTA, typename PointTB> inline pcl::PointXYZ PointAddAssign(const PointTA& __a, const PointTB& __b) {
		pcl::PointXYZ __result{__a.x + __b.x, __a.y + __b.y, __a.z + __b.z};
		return __result;
	}

	/***
	 * @description: __result.geometry = __a.geometry + __b, __result.color = 0x000000;
	 * @param {const PointTA&} __a
	 * @param {const PointTB&} __b
	 * @return {__result}
	 */
	template <typename PointTA, typename PointTB> inline pcl::PointXYZRGB PointAddAssign(const PointTA& __a, const PointTB& __b) {
		pcl::PointXYZRGB __result;
		__result.x = __a.x, __result.y = __a.y, __result.z = __a.z;
		__result.x += __b.x, __result.y += __b.y, __result.z += __b.z;
		return __result;
	}

	/***
	 * @description: __a.geometry += __b.geometry
	 * @param {PointTA&} __a
	 * @param {const PointTB&} __b
	 * @return {*}
	 */
	template <typename PointTA, typename PointTB> inline void PointAddCopy(PointTA& __a, const PointTB& __b) {
		__a.x += __b.x, __a.y += __b.y, __a.z += __b.z;
	}

	/***
	 * @description: __result = __a.geometry - __b.geometry
	 * @param {const PointTA&} __a
	 * @param {const PointTB&} __b
	 * @return {__result}
	 */
	template <typename PointTA, typename PointTB> inline pcl::PointXYZ PointSubAssign(const PointTA& __a, const PointTB& __b) {
		pcl::PointXYZ __result{__a.x - __b.x, __a.y - __b.y, __a.z - __b.z};
		return __result;
	}

	/***
	 * @description: __result.geometry = __a.geometry - __b.geometry, __result.color = 0x000000
	 * @param {const PointTA&} __a
	 * @param {const PointTB&} __b
	 * @return {__result}
	 */
	template <typename PointTA, typename PointTB> inline pcl::PointXYZRGB PointSubAssignColor(const PointTA& __a, const PointTB& __b) {
		pcl::PointXYZRGB __result;
		__result.x = __a.x, __result.y = __a.y, __result.z = __a.z;
		__result.x -= __b.x, __result.y -= __b.y, __result.z -= __b.z;
		return __result;
	}

	/***
	 * @description: __a.geometry -= __b.geometry
	 * @param {PointTA&} __a
	 * @param {const PointTB&} __b
	 * @return {*}
	 */
	template <typename PointTA, typename PointTB> inline void PointSubCopy(PointTA& __a, const PointTB& __b) {
		__a.x -= __b.x, __a.y -= __b.y, __a.z -= __b.z;
	}

	/***
	 * @description: __a.geometry /= __b
	 * @param {PointT&} __a
	 * @param {size_t} __b
	 * @return {*}
	 */
	template <typename PointT> inline void PointDivCopy(PointT& __a, const size_t __b) {
		__a.x /= __b, __a.y /= __b, __a.z /= __b;
	}

	/***
	 * @description: __result = __matrix , __result(:3) += __vector
	 * @param {const Matrix4f&} __matrix
	 * @param {const PointT&} __vector
	 * @return {__result}
	 */
	template <typename PointT> inline Eigen::Matrix4f MatrixAddAssign(const Eigen::Matrix4f& __matrix, const PointT& __vector) {
		Eigen::Matrix4f __result{__matrix};
		__result(0, 3) += __vector.x;
		__result(1, 3) += __vector.y;
		__result(2, 3) += __vector.z;
		return __result;
	}

	/***
	 * @description: __matrix(:3) += __vector
	 * @param {const Matrix4f&} __matrix
	 * @param {const PointT&} __vector
	 * @return {*}
	 */
	template <typename PointT> inline void MatrixAddCopy(Eigen::Matrix4f& __matrix, const PointT& __vector) {
		__matrix(0, 3) += __vector.x;
		__matrix(1, 3) += __vector.y;
		__matrix(2, 3) += __vector.z;
	}

	/***
	 * @description: __point_cloud = __matrix * __point_cloud
	 * @param {PointCloud<PointXYZRGB>} __point_cloud
	 * @param {const Matrix4f&} __matrix
	 * @return {*}
	 */
	inline void PointCloudMul(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const Eigen::Matrix4f& __matrix) {
		for (size_t i = 0; i < __point_cloud.size(); i++) {
			pcl::PointXYZRGB temp{__point_cloud[i]};
			temp.x           = __point_cloud[i].x * __matrix(0, 0) + __point_cloud[i].y * __matrix(0, 1) + __point_cloud[i].z * __matrix(0, 2) + __matrix(0, 3);
			temp.y           = __point_cloud[i].x * __matrix(1, 0) + __point_cloud[i].y * __matrix(1, 1) + __point_cloud[i].z * __matrix(1, 2) + __matrix(1, 3);
			temp.z           = __point_cloud[i].x * __matrix(2, 0) + __point_cloud[i].y * __matrix(2, 1) + __point_cloud[i].z * __matrix(2, 2) + __matrix(2, 3);
			__point_cloud[i] = temp;
		}
	}

	/***
	 * @description: __point_cloud_x += __matrix * __point_cloud_y
	 * @param {PointCloud} __point_cloud_x
	 * @param {const PointCloud} __point_cloud_y
	 * @param {const Matrix4f&} __matrix
	 * @return {*}
	 */
	inline void PointCloudMulAdd(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud_x, const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud_y, const Eigen::Matrix4f& __matrix) {
		for (size_t i = 0; i < __point_cloud_y.size(); i++) {
			pcl::PointXYZRGB temp{__point_cloud_y[i]};
			temp.x = __point_cloud_y[i].x * __matrix(0, 0) + __point_cloud_y[i].y * __matrix(0, 1) + __point_cloud_y[i].z * __matrix(0, 2) + __matrix(0, 3);
			temp.y = __point_cloud_y[i].x * __matrix(1, 0) + __point_cloud_y[i].y * __matrix(1, 1) + __point_cloud_y[i].z * __matrix(1, 2) + __matrix(1, 3);
			temp.z = __point_cloud_y[i].x * __matrix(2, 0) + __point_cloud_y[i].y * __matrix(2, 1) + __point_cloud_y[i].z * __matrix(2, 2) + __matrix(2, 3);
			__point_cloud_x.push_back(temp);
		}
	}

	/***
	 * @description: __point_cloud[i] += __center
	 * @param {PointCloud<PointXYZRGB>} __point_cloud
	 * @param {const PointT&} __center
	 * @return {*}
	 */
	template <typename PointT> inline void PointCloudAdd(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const PointT& __center) {
		for (size_t i = 0; i < __point_cloud.size(); i++) {
			PointAddCopy<pcl::PointXYZRGB, PointT>(__point_cloud[i], __center);
		}
	}

	/***
	 * @description: return if all elements in __group_of_frames are all empty
	 * @param {vector<vector<size_t>>} __group_of_frames
	 * @return {bool}
	 */
	inline bool AreFramesEmpty(const std::vector<std::vector<size_t>>& __group_of_frames) {
		for (size_t i = 0; i < __group_of_frames.size(); i++) {
			if (!__group_of_frames[i].empty()) {
				return false;
			}
		}
		return true;
	}

	/***
	 * @description: return a subnode center according to position and resolution
	 * @param {PointXYZ} __old_center
	 * @param {int} __position
	 * @param {float} __resolution
	 * @return {PointXYZ} __new_center
	 */
	inline pcl::PointXYZ SubnodeCenter(const pcl::PointXYZ __old_center, const int __position, const float __resolution) {
		pcl::PointXYZ __new_center;
		/*
		    for a cube, eight subnodes are
		      6————4
		     /|   /|
		    2—+——0 |
		    | 7——+-5
		    |/   |/
		    3————1
		    xyz : 000 ? 111 (x/y/z > center ? 0 : 1)
		*/
		switch (__position) {
			case 0: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y + __resolution / 2, __old_center.z + __resolution / 2); break;
			case 1: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y + __resolution / 2, __old_center.z - __resolution / 2); break;
			case 2: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y - __resolution / 2, __old_center.z + __resolution / 2); break;
			case 3: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y - __resolution / 2, __old_center.z - __resolution / 2); break;
			case 4: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y + __resolution / 2, __old_center.z + __resolution / 2); break;
			case 5: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y + __resolution / 2, __old_center.z - __resolution / 2); break;
			case 6: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y - __resolution / 2, __old_center.z + __resolution / 2); break;
			case 7: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y - __resolution / 2, __old_center.z - __resolution / 2); break;
		}
		return __new_center;
	}

	/***
	 * @description: convert a 8bit bool vector to uchar, from vector[0] to vector[7] is higher bit to lower bit
	 * @param {vector<bool>} __vector
	 * @return {uint8_t} __result
	 */
	inline uint8_t BoolSetToUChar(const std::vector<bool>& __vector) {
		try {
			uint8_t __result = 0x00;
			if (__vector.size() == 8) {
				for (size_t i = 0; i < 8; i++) {
					__result <<= 1;
					__result |= __vector[i];
				}
				return __result;
			}
			else {
				throw "Size of bool set is not 8.";
			}
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in function BoolSetToUChar() : " << error_message << std::endl;
			std::exit(1);
		}
	}

	/***
	 * @description: count how many 1 in __node_value
	 * @param {uint8_t} __node_value
	 * @return {size_t} __count;
	 */
	inline size_t NodePointCount(uint8_t __node_value) {
		size_t __count = 0;
		for (size_t i = 0; i < 8; i++) {
			__count += (__node_value & 0x01);
			__node_value >>= 1;
		}
		return __count;
	}

	/***
	 * @description: add occupation bit to __bit_map accoring to the __node_value and __merge_node_value
	 * @param {uint8_t} __node_value
	 * @param {uint8_t} __merge_node_value
	 * @param {string&} __bit_map
	 * @return {*}
	 */
	inline void SetBitMap(uint8_t __node_value, uint8_t __merge_node_value, std::string& __bit_map) {
		try {
			// __merge_node_value cannot be 0000 0000
			if (__merge_node_value == 0x00) {
				throw "Empty node.";
			}
			else {
				for (size_t i = 0; i < 8; i++) {
					// extract the highest bit
					bool __merge_bit = (__merge_node_value & 0x80) == 0x00 ? false : true;
					bool __node_bit  = (__node_value & 0x80) == 0x00 ? false : true;
					// merge bit must be 1 to indicate there is a point
					if (__merge_bit) {
						// this point is from this node
						if (__node_bit) {
							__bit_map += '1';
						}
						// this point is from other nodes
						else {
							__bit_map += '0';
						}
					}
					// left move 1 bit to extract next bit
					__merge_node_value <<= 1, __node_value <<= 1;
				}
			}
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error : " << error_message << std::endl;
			std::exit(1);
		}
	}

	/***
	 * @description: from high to low count the 1-bit position, e.g. 11001101 -> 0 1 4 5 7
	 * @param {uint8_t} __node
	 * @param {vector<size_t>} __pos
	 * @return {*}
	 */
	inline void NodePointPosition(uint8_t __node, std::vector<size_t>& __pos) {
		__pos.clear();
		for (size_t i = 0; i < 8; i++) {
			if ((__node & 0x80) != 0) {
				__pos.emplace_back(i);
			}
			__node <<= 1;
		}
	}

	/***
	 * @description: reconstruct point according to the center and position
	 * @param {PointXYZ&} __center
	 * @param {size_t} __pos
	 * @param {PointXYZ&} __point
	 * @return {*}
	 */
	inline void SubnodePoint(const pcl::PointXYZ& __center, const size_t __pos, pcl::PointXYZ& __point) {
		try {
			__point.x = __center.x, __point.y = __center.y, __point.z = __center.z;
			/*
			    for a cube, eight point are
			      6————4
			     /|   /|
			    2—+——0 |
			    | 7——+-5
			    |/   |/
			    3————1
			    7 : is the center location, each edge is 1.0f
			    xyz : 000 ? 111 (x/y/z > center ? 0 : 1)
			*/
			switch (__pos) {
				case 0: __point.x += 1.0f, __point.y += 1.0f, __point.z += 1.0f; break;
				case 1: __point.x += 1.0f, __point.y += 1.0f; break;
				case 2: __point.x += 1.0f, __point.z += 1.0f; break;
				case 3: __point.x += 1.0f; break;
				case 4: __point.y += 1.0f, __point.z += 1.0f; break;
				case 5: __point.y += 1.0f; break;
				case 6: __point.z += 1.0f; break;
				case 7: break;
				default: throw "Position must be 0-7.";
			}
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in function SubnodePoint() : " << error_message << std::endl;
			std::exit(1);
		}
	}

	/***
	 * @description: calculate geometry PSNR between point cloud __x and __y
	 * @param {PointCloud<PointT>&} __x
	 * @param {PointCloud<PointT>&} __y
	 * @return {float} PSNRG
	 */
	template <typename PointT> float PSNRGeo(const pcl::PointCloud<PointT>& __x, const pcl::PointCloud<PointT>& __y) {
		// get the bounding box for point cloud __x and __y
		float maxx = FLT_MIN, maxy = FLT_MIN, maxz = FLT_MIN;
		float minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
		for (auto& i : __x) {
			if (i.x > maxx)
				maxx = i.x;
			if (i.x < minx)
				minx = i.x;
			if (i.y > maxy)
				maxy = i.y;
			if (i.y < miny)
				miny = i.y;
			if (i.z > maxz)
				maxz = i.z;
			if (i.z < minz)
				minz = i.z;
		}

		float range_x = std::max(std::max(maxx - minx, maxy - miny), maxz - minz);

		maxx = FLT_MIN, maxy = FLT_MIN, maxz = FLT_MIN;
		minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
		for (auto& i : __y) {
			if (i.x > maxx)
				maxx = i.x;
			if (i.x < minx)
				minx = i.x;
			if (i.y > maxy)
				maxy = i.y;
			if (i.y < miny)
				miny = i.y;
			if (i.z > maxz)
				maxz = i.z;
			if (i.z < minz)
				minz = i.z;
		}

		float range_y = std::max(std::max(maxx - minx, maxy - miny), maxz - minz);

		// calculate the mse
		float x_mse = 0.0f, y_mse = 0.0f;

		pcl::search::KdTree<PointT> tree_x;
		pcl::search::KdTree<PointT> tree_y;
		tree_x.setInputCloud(__x.makeShared());
		tree_y.setInputCloud(__y.makeShared());

		for (auto& i : __x) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			tree_y.nearestKSearch(i, 1, idx, dis);
			x_mse += dis[0];
		}

		for (auto& i : __y) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			tree_x.nearestKSearch(i, 1, idx, dis);
			y_mse += dis[0];
		}

		x_mse /= __x.size(), y_mse /= __y.size();
		printf("Mse : %.3f %.3f\n", x_mse, y_mse);

		float info = std::max(range_x, range_y);
		printf("Info : %.3f\n", info);
		// calculate psnr
		float psnr = 10.0f * std::log10(info * info / std::max(x_mse, y_mse));
		return psnr;
	}

	/***
	 * @description: calculate YUV difference between RGB points __x and __y
	 * @param {PointXYZRGB&} __x
	 * @param {PointXYZRGB&} __y
	 * @return {Vector3f} result
	 */
	inline Eigen::Vector3f PointColorDifference(const pcl::PointXYZRGB& __x, const pcl::PointXYZRGB& __y) {
		Eigen::Vector3f result;

		float __x_y = 0.299f * static_cast<int>(__x.r) + 0.587f * static_cast<int>(__x.g) + 0.114f * static_cast<int>(__x.b);
		float __x_u = -0.168736f * static_cast<int>(__x.r) - 0.331264f * static_cast<int>(__x.g) + 0.5f * static_cast<int>(__x.b) + 128;
		float __x_v = 0.5f * static_cast<int>(__x.r) - 0.418688f * static_cast<int>(__x.g) - 0.081312f * static_cast<int>(__x.b) + 128;

		float __y_y = 0.299f * static_cast<int>(__y.r) + 0.587f * static_cast<int>(__y.g) + 0.114f * static_cast<int>(__y.b);
		float __y_u = -0.168736f * static_cast<int>(__y.r) - 0.331264f * static_cast<int>(__y.g) + 0.5f * static_cast<int>(__y.b) + 128;
		float __y_v = 0.5f * static_cast<int>(__y.r) - 0.418688f * static_cast<int>(__y.g) - 0.081312f * static_cast<int>(__y.b) + 128;

		result(0) = std::pow(__x_y - __y_y, 2);
		result(1) = std::pow(__x_u - __y_u, 2);
		result(2) = std::pow(__x_v - __y_v, 2);

		return result;
	}

	/***
	 * @description: calculate YUV PSNR between RGB colored point cloud __x and __y
	 * @param {PointCloud<PointXYZRGB>&} __x
	 * @param {PointCloud<PointXYZRGB>&} __y
	 * @return {Vector3f} YUV PSNR
	 */
	inline Eigen::Vector3f PSNRColor(const pcl::PointCloud<pcl::PointXYZRGB>& __x, const pcl::PointCloud<pcl::PointXYZRGB>& __y) {
		float y_mse = 0.0f, u_mse = 0.0f, v_mse = 0.0f;
		float y_mse1 = 0.0f, u_mse1 = 0.0f, v_mse1 = 0.0f;

		pcl::search::KdTree<pcl::PointXYZRGB> tree_x;
		pcl::search::KdTree<pcl::PointXYZRGB> tree_y;

		tree_x.setInputCloud(__x.makeShared());
		tree_y.setInputCloud(__y.makeShared());

		for (auto& i : __x) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			tree_y.nearestKSearch(i, 1, idx, dis);
			Eigen::Vector3f error = operation::PointColorDifference(i, __y[idx[0]]);
			y_mse += error(0), u_mse += error(1), v_mse += error(2);
		}

		y_mse /= __x.size(), u_mse /= __x.size(), v_mse /= __x.size();

		for (auto& i : __y) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			tree_x.nearestKSearch(i, 1, idx, dis);
			Eigen::Vector3f error = operation::PointColorDifference(i, __x[idx[0]]);
			y_mse1 += error(0), u_mse1 += error(1), v_mse1 += error(2);
		}

		y_mse1 /= __x.size(), u_mse1 /= __x.size(), v_mse1 /= __x.size();

		Eigen::Vector3f result;
		result(0) = 10 * std::log10(255 * 255 / std::max(y_mse, y_mse1));
		result(1) = 10 * std::log10(255 * 255 / std::max(u_mse, u_mse1));
		result(2) = 10 * std::log10(255 * 255 / std::max(v_mse, v_mse1));

		printf("MSE of Y component is : %.3f %.3f\n", y_mse, y_mse1);
		printf("MSE of U component is : %.3f %.3f\n", u_mse, u_mse1);
		printf("MSE of V component is : %.3f %.3f\n", v_mse, v_mse1);
		return result;
	}

	/***
	 * @description: __y ^= __x
	 * @param {string&} __x
	 * @param {string&} __y
	 * @return {*}
	 */
	inline void BitMapXOR(const std::string& __x, std::string& __y) {
		try {
			if (__x.size() != __y.size()) {
				std::string error = "Unmatching bitmap size " + std::to_string(__x.size()) + " and " + std::to_string(__x.size()) + "!";
				throw error.c_str();
			}
			for (size_t i = 0; i < __x.size(); i++) {
				__y[i] ^= __x[i];
			}
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in function BitMapXOR : " << error_message << std::endl;
			std::exit(1);
		}
	}

	inline Eigen::Vector3i Index2XYZ8(size_t index) {
		try {
			if (index >= 512) {
				throw "Out of range.";
			}
			int x = 0, y = 0, z = 0;
			for (int i = 0; i < 3; i++) {
				x <<= 1, y <<= 1, z <<= 1;
				z |= (index & 0x1);
				index >>= 1;
				y |= (index & 0x1);
				index >>= 1;
				x |= (index & 0x1);
				index >>= 1;
			}
			Eigen::Vector3i result;
			result(0) = x, result(1) = y, result(2) = z;
			return result;
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in Index2XYZ : " << error_message << std::endl;
			std::exit(1);
		}
	}

	inline Eigen::Vector3i Index2XYZ4(size_t index) {
		try {
			if (index >= 64) {
				throw "Out of range.";
			}
			int x = 0, y = 0, z = 0;
			for (int i = 0; i < 2; i++) {
				x <<= 1, y <<= 1, z <<= 1;
				z |= (index & 0x1);
				index >>= 1;
				y |= (index & 0x1);
				index >>= 1;
				x |= (index & 0x1);
				index >>= 1;
			}
			Eigen::Vector3i result;
			result(0) = x, result(1) = y, result(2) = z;
			return result;
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in Index2XYZ : " << error_message << std::endl;
			std::exit(1);
		}
	}

	inline size_t XYZ2Index8(size_t x, size_t y, size_t z) {
		try {
			if (x >= 8 || y >= 8 || z >= 8) {
				throw "Out of range.";
			}
			size_t temp = 0;
			for (size_t i = 0; i < 3; i++) {
				temp <<= 1;
				temp |= (z & 0x1);
				temp <<= 1;
				temp |= (y & 0x1);
				temp <<= 1;
				temp |= (x & 0x1);
				x >>= 1, y >>= 1, z >>= 1;
			}

			size_t index = 0;
			for (size_t i = 0; i < 9; i++) {
				index <<= 1;
				index |= (temp & 0x1);
				temp >>= 1;
			}
			return index;
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in XYZ2Index : " << error_message << std::endl;
			std::exit(1);
		}
	}

	inline size_t XYZ2Index4(size_t x, size_t y, size_t z) {
		try {
			if (x >= 4 || y >= 4 || z >= 4) {
				throw "Out of range.";
			}
			size_t temp = 0;
			for (size_t i = 0; i < 2; i++) {
				temp <<= 1;
				temp |= (z & 0x1);
				temp <<= 1;
				temp |= (y & 0x1);
				temp <<= 1;
				temp |= (x & 0x1);
				x >>= 1, y >>= 1, z >>= 1;
			}

			size_t index = 0;
			for (size_t i = 0; i < 6; i++) {
				index <<= 1;
				index |= (temp & 0x1);
				temp >>= 1;
			}
			return index;
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in XYZ2Index : " << error_message << std::endl;
			std::exit(1);
		}
	}

	inline float Orth(size_t index) {
		if (index == 0) {
			return 1.0f / std::sqrt(2.0f);
		}
		else {
			return 1.0f;
		}
	}

	/***
	 * @description: convert __x to 8-bit char
	 * @param {int} __x
	 * @return {char} result = -127(__x in(n.a., -127]), __x(__x in (-127, 127)), 127(__x in [127, n.a.))
	 */
	inline char Int2Char(int __x) {
		if (__x <= -127) {
			__x = -127;
		}
		else if (__x >= 127) {
			__x = 127;
		}
		return static_cast<char>(__x);
	}

	/***
	 * @description: convert __x to 16-bit number and return the lower 8-bit
	 * @param {int} __x
	 * @return {char}
	 */
	inline char Int2CharL(int __x) {
		int16_t y;
		if (__x <= -32768) {
			y = -32767;
		}
		else if (__x >= 32767) {
			y = 32767;
		}
		else {
			y = static_cast<int16_t>(__x);
		}
		return static_cast<char>(y & 0x00ff);
	}

	/***
	 * @description: convert __x to 16-bit number and return the higher 8-bit
	 * @param {int} __x
	 * @return {char}
	 */
	inline char Int2CharH(int __x) {
		int16_t y;
		if (__x <= -32768) {
			y = -32767;
		}
		else if (__x >= 32767) {
			y = 32767;
		}
		else {
			y = static_cast<int16_t>(__x);
		}

		y >>= 8;
		return static_cast<char>(y);
	}

	inline int Char2Int(char __high, char __low) {
		int16_t y = 0x0000;
		y |= (__high & 0x00ff);
		y <<= 8;
		y |= (__low & 0x00ff);
		return static_cast<int>(y);
	}

	inline float Char2Float4(char c0, char c1, char c2, char c3) {
		int result = 0x00000000;
		result |= (0x000000ff & c0), result <<= 8;
		result |= (0x000000ff & c1), result <<= 8;
		result |= (0x000000ff & c2), result <<= 8;
		result |= (0x000000ff & c3);
		return static_cast<float>(result);
	}

	inline int Char2Int4(char c0, char c1, char c2, char c3) {
		int result = 0x00000000;
		result |= (0x000000ff & c0), result <<= 8;
		result |= (0x000000ff & c1), result <<= 8;
		result |= (0x000000ff & c2), result <<= 8;
		result |= (0x000000ff & c3);
		return result;
	}

	/***
	 * @description: scale x to y
	 * @param {*}
	 * @return {*}
	 */
	inline Eigen::Vector3f PointCloudScale(pcl::PointCloud<pcl::PointXYZRGB>& __x, pcl::PointCloud<pcl::PointXYZRGB>& __y) {
		// get the bounding box for point cloud __x and __y
		float maxx = FLT_MIN, maxy = FLT_MIN, maxz = FLT_MIN;
		float minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
		for (auto& i : __x) {
			if (i.x > maxx)
				maxx = i.x;
			if (i.x < minx)
				minx = i.x;
			if (i.y > maxy)
				maxy = i.y;
			if (i.y < miny)
				miny = i.y;
			if (i.z > maxz)
				maxz = i.z;
			if (i.z < minz)
				minz = i.z;
		}
		Eigen::Vector3f range_x;
		range_x(0) = maxx - minx, range_x(1) = maxy - miny, range_x(2) = maxz - minz;

		maxx = FLT_MIN, maxy = FLT_MIN, maxz = FLT_MIN;
		minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
		for (auto& i : __y) {
			if (i.x > maxx)
				maxx = i.x;
			if (i.x < minx)
				minx = i.x;
			if (i.y > maxy)
				maxy = i.y;
			if (i.y < miny)
				miny = i.y;
			if (i.z > maxz)
				maxz = i.z;
			if (i.z < minz)
				minz = i.z;
		}

		Eigen::Vector3f range_y;
		range_y(0) = maxx - minx, range_y(1) = maxy - miny, range_y(2) = maxz - minz;

		Eigen::Vector3f result;
		result(0) = range_y(0) / range_x(0), result(1) = range_y(1) / range_x(1), result(2) = range_y(2) / range_x(2);
		return result;
	}

	inline void MatrixScale(Eigen::Matrix4f& __x, Eigen::Vector3f& __scale) {
		__x(0, 0) *= __scale(0), __x(0, 1) *= __scale(0), __x(0, 2) *= __scale(0), __x(0, 3) *= __scale(0);
		__x(1, 0) *= __scale(1), __x(1, 1) *= __scale(1), __x(1, 2) *= __scale(1), __x(1, 3) *= __scale(1);
		__x(2, 0) *= __scale(2), __x(2, 1) *= __scale(2), __x(2, 2) *= __scale(2), __x(2, 3) *= __scale(2);
	}
}  // namespace operation

}  // namespace vvs
#endif