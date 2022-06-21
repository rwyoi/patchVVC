/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 15:13:59
 * @LastEditTime: 2022-06-21 18:51:35
 * @LastEditors: ChenRP07
 * @Description:
 */
#ifndef _LIB_REGISTRATION_H_
#define _LIB_REGISTRATION_H_
#include "coder/operation.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <thread>

namespace vvs {
namespace registration {
	// base class
	class regist_base {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB> source_point_cloud_;    // point cloud to be transformed
		pcl::PointCloud<pcl::PointXYZRGB> target_point_cloud_;    // point cloud to be aligned with
		pcl::PointCloud<pcl::PointXYZRGB> result_point_cloud_;    // align result
		Eigen::Matrix4f                   tranformation_matrix_;  // transformation matrix
		float                             mean_squred_error_;     // align score

	  public:
		/***
		 * @description: constructor, set motion vector to an identity matrix
		 * @param {*}
		 * @return {*}
		 */
		regist_base();

		// virtual function
		virtual bool align(const int = 0) = 0;

		/***
		 * @description: get mean_squred_error_
		 * @param {*}
		 * @return {float} mean_squred_error_
		 */
		float GetMSE() const;

		/***
		 * @description: set source point cloud by swapping
		 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} __point_cloud
		 * @return {*}
		 */
		void SetSourcePointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: set source point cloud by copying
		 * @param {const pcl::PointCloud<pcl::PointXYZRGB>&} __point_cloud
		 * @return {*}
		 */
		void SetSourcePointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: set target point cloud by swapping
		 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} __point_cloud
		 * @return {*}
		 */
		void SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: set target point cloud by copying
		 * @param {const pcl::PointCloud<pcl::PointXYZRGB>&} __point_cloud
		 * @return {*}
		 */
		void SetTargetPointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: get motion vector
		 * @param {*}
		 * @return {Matrix4f} tranformation_matrix_
		 */
		Eigen::Matrix4f GetMotionVector() const;

		/***
		 * @description: get source point cloud, using swapping
		 * @param {pcl::PointCloud<pcl::PointXYZRGB>} __point_cloud
		 * @return {*}
		 */
		void GetSourcePointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: constructor, set motion vector to an identity matrix
		 * @param {*}
		 * @return {*}
		 */
		void GetSourcePointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) const;

		/***
		 * @description: get target point cloud, using swapping
		 * @param {PointCloud} __point_cloud
		 * @return {*}
		 */
		void GetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: get target point cloud, using copying
		 * @param {PointCloud} __point_cloud
		 * @return {*}
		 */
		void GetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) const;

		/***
		 * @description: get source size
		 * @param {*}
		 * @return {size_t}
		 */
		size_t GetSourcePointCloudSize() const;

		/***
		 * @description: get target size
		 * @param {*}
		 * @return {size_t}
		 */
		size_t GetTargetPointCloudSize() const;

		/***
		 * @description: get result point cloud, using swapping
		 * @param {PointCloud} __point_cloud
		 * @return {*}
		 */
		void GetResultPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: get result point cloud, using copying
		 * @param {PointCloud} __point_cloud
		 * @return {*}
		 */
		void GetResultPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) const;

		/***
		 * @description: move source cloud to align centroid with target cloud, save the translated cloud in result
		 * @param {*}
		 * @return {*}
		 */
		void GlobalCentroidAlignment();

		/***
		 * @description: move source cloud to align centroid with target cloud, save the translated cloud in result, using k-nn
		 * @param {*}
		 * @return {*}
		 */
		void LocalCentroidAlignment();
	};

	// icp
	class ICP : public regist_base {
	  private:
		const float kCorrespondenceThreshold;  // max correspondence distance
		const float kMSEThreshold;             // max mse difference between two iteration
		const float kTransformationThreshold;  // max matrix difference between two iteration
		const int   kIterationThreshold;       // max iteration

	  public:
		/***
		 * @description: constructor
		 * @param {float} max_correspondence
		 * @param {int} max_iteration
		 * @param {float} max_mse_difference
		 * @param {float} max_tranformation_difference
		 * @return {*}
		 */
		ICP(const float max_correspondence, const int max_iteration, const float max_mse_difference = 0.01f, const float max_transformation_difference = 1e-6);

		/***
		 * @description: do Iterative Closest Point Algorithm, using centroid alignment
		 * @param {int} centroid_alignment_type, 0 none, 1 global, 2 local.
		 * @return {bool} Algorithm converged ?
		 */
		bool align(const int centroid_alignment_type = 0);
	};

	class NICP : public regist_base {
	  private:
		const float                             kCorrespondenceThreshold;      // max correspondence distance
		const float                             kMSEThreshold;                 // max mse difference between two iteration
		const float                             kTransformationThreshold;      // max matrix difference between two iteration
		const int                               kIterationThreshold;           // max matrix difference between two iteration
		const float                             kNormalRadius;                 // max iteration
		pcl::PointCloud<pcl::PointXYZRGBNormal> normaled_result_point_cloud_;  // result point cloud with normals
		pcl::PointCloud<pcl::PointXYZRGBNormal> normaled_target_point_cloud_;  // target point cloud with normals

	  protected:
		/***
		 * @description: compute normals
		 * @param {*}
		 * @return {*}
		 */
		void ComputeNormals();

	  public:
		/***
		 * @description: constructor
		 * @param {float} max_correspondence
		 * @param {int} max_iteration
		 * @param {float} normal_search_radius
		 * @param {float} max_mse_difference
		 * @param {float} max_tranformation_diference
		 * @return {*}
		 */
		NICP(const float max_correspondence, const int, const float = 10.0f, const float = 0.01f, const float = 1e-6);

		/***
		 * @description: do normal icp, using centroid alignment
		 * @param {int} centroid_alignment_type, 0 none, 1 global, 2 local
		 * @return {*}
		 */
		bool align(const int = 0);
	};

	class ParallelICP {
	  private:
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> source_patches_;
		pcl::PointCloud<pcl::PointXYZRGB>              target_point_cloud_;
		const float                                    kCorrespondenceThreshold;  // max correspondence distance
		const float                                    kMSEThreshold;             // max mse difference between two iteration
		const float                                    kTransformationThreshold;  // max matrix difference between two iteration
		const int                                      kIterationThreshold;       // max iteration
		const int                                      kThreads;                  // thread number
		std::vector<int>                               target_patch_index_;
		std::vector<Eigen::Matrix4f>                   motion_vectors_;
		std::vector<float>                             mean_squred_errors_;
		std::queue<size_t>                             task_pool_;
		std::mutex                                     task_pool_mutex_;
		std::mutex                                     IO_mutex_;
		bool                                           indexed = false;
		Eigen::Matrix4f                                global_vector_;

	  public:
		ParallelICP(const int, const float, const int, const float = 0.01f, const float = 1e-6);
		void ThreadProcess();
		void ParallelAlign();
		void SetSourcePatchesCopy(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&);
		void SetSourcePatchesSwap(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&);
		void SetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>&);
		void SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);
		void GetTargetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&, std::vector<Eigen::Matrix4f>&);
		void GetTargetMSEs(std::vector<float>&) const;
	};
}  // namespace registration
}  // namespace vvs
#endif