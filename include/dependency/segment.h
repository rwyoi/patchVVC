/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 14:56:35
 * @LastEditTime: 2022-06-30 15:57:53
 * @LastEditors: ChenRP07
 * @Description: Header of Point Cloud Segmentation.
 */
#ifndef _LIB_SEGMENT_H_
#define _LIB_SEGMENT_H_
#include <cfloat>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <queue>

namespace vvs {
namespace segment {
	/*
	 * class Patch : base for the different clustering method.
	 */
	class Patch {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB> point_cloud_;  // unsplit point cloud
		std::vector<int>                  point_index_;  // patch index for each point
	  public:
		/***
		 * @description: constructor for Patch
		 * @param {null}
		 * @return {Patch}
		 */
		Patch();

		/***
		 * @description: set point_cloud using vetor copy
		 * @param {source point cloud \src_}
		 * @return {null}
		 */
		void SetPointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>& src_);

		/***
		 * @description: set point cloud using vector swap,
		    after this the src_ will be exchanged with point_cloud_
		 * @param {source point cloud \src_}
		 * @return {null}
		 */
		void SetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& src_);

		/***
		 * @description: output point cloud
		 * @param {output target \dst_}
		 * @return {null}
		 */
		void GetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& dst_);

		/***
		 * @description: output patch index
		 * @param {output target \dst_}
		 * @return {max index}
		 */
		int GetPointIndex(std::vector<int>& dst_);

		/***
		 * @description: output patches
		 * @param {output target \dst_}
		 * @return {max index}
		 */
		int          GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& dst_);
		virtual void Clustering() = 0;
	};

	/* simple method -- constant clustering */
	class SimplePatch : public Patch {
	  private:
		const size_t kPatchNumber;  // patch number

	  public:
		/***
		 * @description: constructor, set patch number
		 * @param {int} num
		 * @return {null}
		 */
		SimplePatch(const size_t __patch_number = 50);

		/***
		 * @description: using a constant and compact clustering method to segment point_cloud_ to kPatchNumber patches.
		 * @param {*}
		 * @return {*}
		 */
		void Clustering();
	};
}  // namespace segment
}  // namespace vvs
#endif