/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 15:14:31
 * @LastEditTime: 2022-06-21 15:30:28
 * @LastEditors: ChenRP07
 * @Description:
 */
#ifndef _LIB_OPERATION_H_
#define _LIB_OPERATION_H_
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
}  // namespace operation
}  // namespace vvs
#endif