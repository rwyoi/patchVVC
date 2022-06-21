/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 15:03:42
 * @LastEditTime: 2022-06-21 15:12:16
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "coder/segment.h"
using namespace vvs::segment;

/***
 * @description: constructor, set patch number
 * @param {int} num
 * @return {null}
 */
SimplePatch::SimplePatch(const size_t __patch_number) : Patch(), kPatchNumber{__patch_number} {}

/***
 * @description: using a constant and compact clustering method to segment point_cloud_ to kPatchNumber patches.
 * @param {*}
 * @return {*}
 */
void SimplePatch::Clustering() {
	const int kPointPerCluster = std::floor(this->point_cloud_.size() / this->kPatchNumber);

	// compare method for priority_queue
	struct Cmp {
		bool operator()(const pcl::PointCloud<pcl::PointXYZRGB>& x_, const pcl::PointCloud<pcl::PointXYZRGB>& y_) {
			return x_.size() < y_.size();
		}
	};

	std::priority_queue<pcl::PointCloud<pcl::PointXYZRGB>, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>, Cmp> block_queue;

	block_queue.push(this->point_cloud_);

	while (block_queue.top().size() > kPointPerCluster) {
		// divide the largest point cloud
		pcl::PointCloud<pcl::PointXYZRGB> temp_cloud = block_queue.top();
		block_queue.pop();
		pcl::PointCloud<pcl::PointXYZRGB> new_cloud_1, new_cloud_2;
		// decide which dimension to be divided
		float MAX_X = FLT_MIN, MAX_Y = FLT_MIN, MAX_Z = FLT_MIN;
		float MIN_X = FLT_MAX, MIN_Y = FLT_MAX, MIN_Z = FLT_MAX;
		for (auto& i : temp_cloud) {
			MAX_X = std::max(MAX_X, i.x), MIN_X = std::min(MIN_X, i.x);
			MAX_Y = std::max(MAX_Y, i.y), MIN_Y = std::min(MIN_Y, i.y);
			MAX_Z = std::max(MAX_Z, i.z), MIN_Z = std::min(MAX_Z, i.z);
		}

		float MAX_DIM;
		char  DIV_DIM;
		if (MAX_X - MIN_X > MAX_Y - MIN_Y)
			MAX_DIM = MAX_X - MIN_X, DIV_DIM = 'x';
		else
			MAX_DIM = MAX_Y - MIN_Y, DIV_DIM = 'y';
		if (MAX_DIM < MAX_Z - MIN_Z)
			MAX_DIM = MAX_Z - MIN_Z, DIV_DIM = 'z';
		// divide by a dimension
		float DIV_MID;
		if (DIV_DIM == 'x') {
			DIV_MID = MIN_X + MAX_DIM / 2;
			for (auto& i : temp_cloud)
				if (i.x < DIV_MID)
					new_cloud_1.push_back(i);
				else
					new_cloud_2.push_back(i);
		}
		else if (DIV_DIM == 'y') {
			DIV_MID = MIN_Y + MAX_DIM / 2;
			for (auto& i : temp_cloud)
				if (i.y < DIV_MID)
					new_cloud_1.push_back(i);
				else
					new_cloud_2.push_back(i);
		}
		else if (DIV_DIM == 'z') {
			DIV_MID = MIN_Z + MAX_DIM / 2;
			for (auto& i : temp_cloud)
				if (i.z < DIV_MID)
					new_cloud_1.push_back(i);
				else
					new_cloud_2.push_back(i);
		}
		// add results to the queue
		block_queue.push(new_cloud_1);
		block_queue.push(new_cloud_2);
	}

	// calculate the centroids from the largest kPatchNumber clusters
	pcl::PointCloud<pcl::PointXYZRGB> centroids;
	for (int i = 0; i < this->kPatchNumber; i++) {
		pcl::PointXYZRGB                  center;
		pcl::PointCloud<pcl::PointXYZRGB> temp_cloud = block_queue.top();
		block_queue.pop();
		center.x = 0, center.y = 0, center.z = 0;
		for (auto& i : temp_cloud) {
			center.x += i.x;
			center.y += i.y;
			center.z += i.z;
		}
		center.x /= temp_cloud.size();
		center.y /= temp_cloud.size();
		center.z /= temp_cloud.size();

		centroids.push_back(center);
	}

	// do clustering by the centroids
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(centroids.makeShared());
	for (std::size_t i = 0; i < this->point_cloud_.size(); i++) {
		std::vector<int>   index(1);
		std::vector<float> distance(1);
		tree.nearestKSearch(this->point_cloud_[i], 1, index, distance);
		this->point_index_[i] = index[0];
	}
}