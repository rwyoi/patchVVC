/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 15:00:24
 * @LastEditTime: 2022-06-21 15:00:24
 * @LastEditors: ChenRP07
 * @Description:
 */
#include <coder/segment.h>
using namespace vvs::segment;

/***
 * @description: constructor for Patch
 * @param {null}
 * @return {Patch}
 */
Patch::Patch() {
	this->point_cloud_.reserve(10);
	this->point_index_.reserve(10);
}

/***
 * @description: set point cloud using vector swap,
    after this the src_ will be exchanged with point_cloud_
 * @param {source point cloud \src_}
 * @return {null}
 */
void Patch::SetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& src_) {
	this->point_cloud_.swap(src_);
	this->point_index_.resize(this->point_cloud_.size());
}

/***
 * @description: set point_cloud using vetor copy
 * @param {source point cloud \src_}
 * @return {null}
 */
void Patch::SetPointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>& src_) {
	for (auto& i : src_)
		this->point_cloud_.push_back(i);
	this->point_index_.resize(this->point_cloud_.size());
}

/***
 * @description: output point cloud
 * @param {output target \dst_}
 * @return {null}
 */
void Patch::GetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& dst_) {
	for (auto& i : this->point_cloud_)
		dst_.push_back(i);
}

/***
 * @description: output patch index
 * @param {output target \dst_}
 * @return {max index}
 */
int Patch::GetPointIndex(std::vector<int>& dst_) {
	for (auto& i : this->point_index_)
		dst_.push_back(i);
	return *std::max_element(this->point_index_.begin(), this->point_index_.end());
}

/***
 * @description: output patches
 * @param {output target \dst_}
 * @return {max index}
 */
int Patch::GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& dst_) {
	int max_index = *std::max_element(this->point_index_.begin(), this->point_index_.end());
	dst_.resize(max_index + 1);

	for (size_t i = 0; i < this->point_index_.size(); i++)
		dst_[this->point_index_[i]].push_back(this->point_cloud_[i]);

	return max_index;
}
