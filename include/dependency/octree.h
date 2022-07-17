/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 19:55:40
 * @LastEditTime: 2022-07-16 21:43:24
 * @LastEditors: ChenRP07
 * @Description: Header of octree
 */

#ifndef _LIB_OCTREE_H_
#define _LIB_OCTREE_H_
#include "3rd/zstd.h"
#include "dependency/io.h"
#include "dependency/operation.hpp"
#include "dependency/registration.h"
#include "dependency/type.hpp"
#include <cfloat>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define _DCT_FIX_16_
#define _RAHT_FIX_8_
// #define _RAHT_RLE_
#define _RAHT_SUBBANDS_
#define DEFAULT_KQSTEP 10
#define DEFAULT_PKQSTEP 30
namespace vvs {
namespace octree {
	class GOF {
	  private:
		// patches for each frame
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> frame_patches_;
		// motion vectors for each patch
		std::vector<Eigen::Matrix4f> motion_vectors_;

		std::vector<bool>                             patch_coding_mode_;
		std::vector<float>                            patches_mses_;
		pcl::PointCloud<pcl::PointXYZRGB>             fitting_patch_;
		std::vector<std::vector<vvs::type::ColorRGB>> patches_colors_;

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

		/***
		 * @description: color interpolation for each point in fitting patch
		 * @param {int} kInterpolationNumber
		 * @return {*}
		 */
		void PatchColorFitting(const int kInterpolationNumber);

		void Compression(vvs::type::IFramePatch& __i_frame, std::vector<vvs::type::PFramePatch>& __p_frames, size_t index);

		void Output(pcl::PointCloud<pcl::PointXYZRGB>& cloud, size_t index) {
			for (size_t i = 0; i < this->fitting_patch_.size(); i++) {
				pcl::PointXYZRGB p;
				p.x = this->fitting_patch_[i].x, p.y = this->fitting_patch_[i].y, p.z = this->fitting_patch_[i].z;
				p.r = static_cast<uint8_t>(this->patches_colors_[index][i].r_), p.g = static_cast<uint8_t>(this->patches_colors_[index][i].g_),
				p.b = static_cast<uint8_t>(this->patches_colors_[index][i].b_);
				cloud.emplace_back(p);
			}
		}

		bool out = false;
	};

	class Octree3D {
	  private:
		// tree nodes
		std::vector<std::vector<vvs::type::TreeNode>> tree_nodes_;
		// patch colors
		std::vector<std::vector<vvs::type::ColorYUV>> colors_;

		// min resolution
		const float kMinResolution;
		// tree center
		pcl::PointXYZ tree_center_;
		// tree height
		size_t tree_height_;
		// tree max resolution
		float tree_resolution_;
		// gof
		size_t kGroupOfFrames;

	  public:
		/***
		 * @description: constructor, __res is the min resolution, default is 2.0
		 * @param {float} __res
		 * @return {*}
		 */
		Octree3D(const float __res = 2.0f);

		/***
		 * @description: add a tree node on __height layer and this node res is __res, center is __center, point indexes are __node_points
		 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
		 * @param {const vector<vector<ColorRGB>>&} __point_colors
		 * @param {vector<size_t>&} __node_points
		 * @param {size_t} __height
		 * @param {PointXYZ} __center
		 * @param {float} __res
		 * @return {*}
		 */
		bool AddTreeNode(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const std::vector<std::vector<vvs::type::ColorRGB>>& __point_colors, std::vector<size_t>& __node_points,
		                 size_t __height, float __res, pcl::PointXYZ __center);

		/***
		 * @description: use __point_cloud and __point_colors to create a octree, cloud center is __center and span range is __res
		 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
		 * @param {const vector<vector<ColorRGB>>&} __point_colors
		 * @return {*}
		 */
		void SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const std::vector<std::vector<vvs::type::ColorRGB>>& __point_colors);

		/***
		 * @description: using zstd to compress the tree nodes to __result, and get the tree parameters
		 * @param {string&} __result
		 * @param {PointXYZ&} __center
		 * @param {size_t&} __tree_height
		 * @return {*}
		 */
		void TreeCompression(std::string& __result, pcl::PointXYZ& __center, size_t& __tree_height);

		void RAHT(std::string& __result, size_t index, const int kQStep);

		void   ColorCompression(std::vector<std::string>& __result, size_t index, const int kQStep = DEFAULT_KQSTEP);
		bool   out = false;
		size_t size() {
			return this->colors_[0].size();
		}
	};

	class SingleOctree3D {
	  private:
		// tree nodes
		std::vector<std::vector<vvs::type::TreeNode>> tree_nodes_;

		// min resolution
		const float kMinResolution;
		// tree center
		pcl::PointXYZ tree_center_;
		// tree height
		size_t tree_height_;
		// tree max resolution
		float tree_resolution_;

	  public:
		/***
		 * @description: constructor, __res is the min resolution, default is 2.0
		 * @param {float} __res
		 * @return {*}
		 */
		SingleOctree3D(const float __res = 2.0f);

		/***
		 * @description: add a tree node on __height layer and this node res is __res, center is __center, point indexes are __node_points
		 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
		 * @param {const vector<vector<ColorRGB>>&} __point_colors
		 * @param {vector<size_t>&} __node_points
		 * @param {size_t} __height
		 * @param {PointXYZ} __center
		 * @param {float} __res
		 * @return {*}
		 */
		bool AddTreeNode(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, std::vector<size_t>& __node_points, size_t __height, float __res, pcl::PointXYZ __center);

		/***
		 * @description: use __point_cloud and __point_colors to create a octree, cloud center is __center and span range is __res
		 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
		 * @param {const vector<vector<ColorRGB>>&} __point_colors
		 * @return {*}
		 */
		void SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: using zstd to compress the tree nodes to __result, and get the tree parameters
		 * @param {string&} __result
		 * @param {PointXYZ&} __center
		 * @param {size_t&} __tree_height
		 * @return {*}
		 */
		void TreeCompression(std::string& __result, pcl::PointXYZ& __center, size_t& __tree_height);

		void RAHT(std::string& __result, const int kQStep = DEFAULT_KQSTEP);
	};

	// octree for decoding
	class DeOctree3D {
	  private:
		// tree nodes
		std::vector<std::vector<vvs::type::TreeNode>> tree_nodes_;
		// center for each tree nodes
		std::vector<std::vector<pcl::PointXYZ>> tree_points_;

		// minimal resolution
		float kMinResolution;

		// center for whole tree
		pcl::PointXYZ tree_center_;
		// tree height
		size_t tree_height_;
		// maximum resolution
		float tree_resolution_;

	  public:
		/***
		 * @description: constructor
		 * @param {float} __min_res
		 * @return {*}
		 */
		DeOctree3D(const float __min_res = 2.0f);

		/***
		 * @description: set tree center
		 * @param {const PointXYZ&} __center
		 * @return {*}
		 */
		void SetCenter(const pcl::PointXYZ& __center);

		/***
		 * @description: set tree height
		 * @param {const size_t} __height
		 * @return {*}
		 */
		void SetHeight(const size_t __height);

		/***
		 * @description: decompress __tree using zstd, set tree nodes and calculate node centers
		 * @param {string&} __tree
		 * @return {*}
		 */
		void SetTree(std::string& __tree);

		void IRAHT(std::string& __source, size_t point_cnt, std::vector<vvs::type::ColorYUV>& __result, size_t index, const int Qstep = DEFAULT_KQSTEP);

		/***
		 * @description: get patch geometry information
		 * @param {PointCloud<PointXYZ>&} __patch
		 * @return {*}
		 */
		void GetPatch(std::string& __color_source, pcl::PointCloud<pcl::PointXYZ>& __patch, std::vector<vvs::type::ColorYUV>& __color, size_t index);

		bool out = false;
	};

	class FittingOctree3D {
	  private:
		// tree nodes
		pcl::PointCloud<pcl::PointXYZRGB> fitting_patch_;
		size_t                            avg_size_;

		float         tree_resolution_;
		pcl::PointXYZ tree_center_;

	  public:
		/***
		 * @description: constructor, __res is the min resolution, default is 2.0
		 * @param {float} __res
		 * @return {*}
		 */
		FittingOctree3D();

		/***
		 * @description: add a tree node on __height layer and this node res is __res, center is __center, point indexes are __node_points
		 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
		 * @param {const vector<vector<ColorRGB>>&} __point_colors
		 * @param {vector<size_t>&} __node_points
		 * @param {size_t} __height
		 * @param {PointXYZ} __center
		 * @param {float} __res
		 * @return {*}
		 */
		void AddTreeNode(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& __point_cloud, float __res, pcl::PointXYZ __center);

		/***
		 * @description: use __point_cloud and __point_colors to create a octree, cloud center is __center and span range is __res
		 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
		 * @param {const vector<vector<ColorRGB>>&} __point_colors
		 * @return {*}
		 */
		void SetPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& __point_cloud);

		void GetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);
	};

}  // namespace octree
}  // namespace vvs
#endif