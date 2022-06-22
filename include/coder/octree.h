/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 19:55:40
 * @LastEditTime: 2022-06-22 18:41:08
 * @LastEditors: ChenRP07
 * @Description:
 */

#ifndef _LIB_OCTREE_H_
#define _LIB_OCTREE_H_
#include "coder/operation.hpp"
#include "coder/registration.h"
#include <cfloat>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vvs {
namespace octree {
	class GOF {
	  private:
		// patches for each frame
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> frame_patches_;
		// motion vectors for each patch
		std::vector<Eigen::Matrix4f> motion_vectors_;

		std::vector<bool>                          patch_coding_mode_;
		std::vector<float>                         patches_mses_;
		pcl::PointCloud<pcl::PointXYZRGB>          fitting_patch_;
		std::vector<std::vector<vvs::type::Color>> patches_colors_;
		// GOF
		const size_t kGroupOfFrames;
		// how many frames are there now
		size_t frame_number_;
		// bounding box for this GOF
		float min_x_;
		float max_x_;
		float min_y_;
		float max_y_;
		float min_z_;
		float max_z_;

	  public:
		/***
		 * @description: constructor, input kGroupOfFrames
		 * @param {int} group_of_frames
		 * @return {*}
		 */
		GOF(const size_t group_of_frames);

		/***
		 * @description: get size of frame_patches_
		 * @param {*}
		 * @return {size_t} frame_patches_.size()
		 */
		size_t size() const;

		/***
		 * @description: get size of frame_patches_[index]
		 * @param {size_t} index
		 * @return {size_t} frame_patches_[index].size()
		 */
		size_t size(const size_t index) const;

		/***
		 * @description: get i-th frame_patches_'s j-th element
		 * @param {size_t} __x
		 * @param {size_t} __y
		 * @return {PointXYZRGB&} frame_patches_[__x][__y]
		 */
		pcl::PointXYZRGB& operator()(const size_t __x, const size_t __y);

		/***
		 * @description: get i-th frame_patches_'s j-th element
		 * @param {size_t} __x
		 * @param {size_t} __y
		 * @return {PointXYZRGB&} frame_patches_[__x][__y]
		 */
		const pcl::PointXYZRGB& operator()(const size_t __x, const size_t __y) const;

		/***
		 * @description: add a patch into gof
		 * @param {PointCloud} __patch
		 * @param {Matrix4f} __matrix
		 * @return {*}
		 */
		void AddPatch(const pcl::PointCloud<pcl::PointXYZRGB>& __patch, const Eigen::Matrix4f& __matrix);

		/***
		 * @description: get __index-th patch
		 * @param {PointCloud} __patch
		 * @param {Matrix4f} __matrix
		 * @param {int} __index
		 * @return {*}
		 */
		void GetPatch(pcl::PointCloud<pcl::PointXYZRGB>& __patch, Eigen::Matrix4f& __matrix, const size_t __index) const;

		/***
		 * @description: get all patches
		 * @param {vector<PointCloud>} __patches
		 * @param {vector<Matrix4f>} __matrices
		 * @return {*}
		 */
		void GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& __patches, std::vector<Eigen::Matrix4f>& __matrices) const;

		/***
		 * @description: return maximum span of 3D coordinates
		 * @param {*}
		 * @return {float}
		 */
		float GetResolution() const;

		/***
		 * @description: return maximum span of 3D coordinates which is rounded up as a power of 2;
		 * @param {*}
		 * @return {float}
		 */
		float GetResolutionBinary() const;

		/***
		 * @description: return the center of 3D coordinates of patches
		 * @param {*}
		 * @return {PointXYZ}
		 */
		pcl::PointXYZ GetCenter() const;

		/***
		 * @description: get motion vectors
		 * @param {vector<Matrix4f>} __matrices
		 * @return {*}
		 */
		void GetMatrices(std::vector<Eigen::Matrix4f>& __matrices) const;

		/***
		 * @description: use k-means to generate a fitting point cloud patch
		 * @param {float} kMSEThreshold
		 * @param {float} max_correnspondence
		 * @param {int} max_iteration
		 * @return {*}
		 */
		void GenerateFittingPatch(const float kMSEThreshold, const float max_correspondence, const int max_iteration);

		/***
		 * @description: cout each patches' mse
		 * @param {*}
		 * @return {*}
		 */
		void OutputPSNR();

		/***
		 * @description: output the fitting patch
		 * @param {PointCloud<PointXYZRGB>&} __point_cloud
		 * @return {*}
		 */
		void OutputFittingPatch(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);
	};

}  // namespace octree
}  // namespace vvs
#endif