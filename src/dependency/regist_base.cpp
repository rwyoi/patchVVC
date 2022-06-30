/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 15:33:27
 * @LastEditTime: 2022-06-30 15:40:46
 * @LastEditors: ChenRP07
 * @Description: Implement of base class for point cloud registration
 */
#include "dependency/registration.h"
using namespace vvs::registration;

/***
 * @description: constructor, set motion vector to an identity matrix
 * @param {*}
 * @return {*}
 */
regist_base::regist_base() {
	// this->source_point_cloud_.reserve(10);
	// this->target_point_cloud_.reserve(10);
	this->tranformation_matrix_ = Eigen::Matrix4f::Identity();
	this->mean_squred_error_    = -1.0f;
}

/***
 * @description: get mean_squred_error_
 * @param {*}
 * @return {float} mean_squred_error_
 */
float regist_base::GetMSE() const {
	return this->mean_squred_error_;
}

/***
 * @description: set source point cloud by swapping
 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
 * @return {*}
 */
void regist_base::SetSourcePointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	this->source_point_cloud_.swap(point_cloud);
	this->result_point_cloud_.resize(this->source_point_cloud_.size());
	for (size_t i = 0; i < this->source_point_cloud_.size(); i++) {
		this->result_point_cloud_[i] = this->source_point_cloud_[i];
	}
	// this->result_point_cloud_ = this->source_point_cloud_;
}

/***
 * @description: set source point cloud by copying
 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
 * @return {*}
 */
void regist_base::SetSourcePointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	this->source_point_cloud_.resize(point_cloud.size());
	for (size_t i = 0; i < point_cloud.size(); i++) {
		this->source_point_cloud_[i] = point_cloud[i];
	}
	this->result_point_cloud_.resize(this->source_point_cloud_.size());
	for (size_t i = 0; i < this->source_point_cloud_.size(); i++) {
		this->result_point_cloud_[i] = this->source_point_cloud_[i];
	}
	// this->result_point_cloud_ = this->source_point_cloud_;
}

/***
 * @description: set target point cloud by swapping
 * @param {*}
 * @return {*}
 */
void regist_base::SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	this->target_point_cloud_.swap(point_cloud);
}

/***
 * @description: set target point cloud by copying
 * @param {*}
 * @return {*}
 */
void regist_base::SetTargetPointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	this->target_point_cloud_.resize(point_cloud.size());
	for (size_t i = 0; i < point_cloud.size(); i++) {
		this->target_point_cloud_[i] = point_cloud[i];
	}
}

/***
 * @description: get motion vector
 * @param {*}
 * @return {Matrix4f} tranformation_matrix_
 */
Eigen::Matrix4f regist_base::GetMotionVector() const {
	return this->tranformation_matrix_;
}

/***
 * @description: get source point cloud, using swapping
 * @param {PointCloud} point_cloud
 * @return {*}
 */
void regist_base::GetSourcePointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	point_cloud.swap(this->source_point_cloud_);
}

/***
 * @description: get source point cloud, using copying
 * @param {PointCloud} point_cloud
 * @return {*}
 */
void regist_base::GetSourcePointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) const {
	point_cloud.resize(this->source_point_cloud_.size());
	for (size_t i = 0; i < this->source_point_cloud_.size(); i++) {
		point_cloud[i] = this->source_point_cloud_[i];
	}
}

/***
 * @description: get target point cloud, using swapping
 * @param {PointCloud} point_cloud
 * @return {*}
 */
void regist_base::GetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	point_cloud.swap(this->target_point_cloud_);
}

/***
 * @description: get target point cloud, using copying
 * @param {PointCloud} point_cloud
 * @return {*}
 */
void regist_base::GetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) const {
	point_cloud.resize(this->target_point_cloud_.size());
	for (size_t i = 0; i < this->target_point_cloud_.size(); i++) {
		point_cloud[i] = this->target_point_cloud_[i];
	}
}

/***
 * @description: get source size
 * @param {*}
 * @return {size_t}
 */
size_t regist_base::GetSourcePointCloudSize() const {
	return this->source_point_cloud_.size();
}

/***
 * @description: get target size
 * @param {*}
 * @return {size_t}
 */
size_t regist_base::GetTargetPointCloudSize() const {
	return this->target_point_cloud_.size();
}

/***
 * @description: get result point cloud, using swapping
 * @param {PointCloud} __point_cloud
 * @return {*}
 */
void regist_base::GetResultPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) {
	__point_cloud.swap(this->result_point_cloud_);
}

/***
 * @description: get result point cloud, using copying
 * @param {PointCloud} __point_cloud
 * @return {*}
 */
void regist_base::GetResultPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) const {
	__point_cloud.resize(this->result_point_cloud_.size());
	for (size_t i = 0; i < this->result_point_cloud_.size(); i++) {
		__point_cloud[i] = this->result_point_cloud_[i];
	}
}

/***
 * @description: move source cloud to align centroid with target cloud, save the translated cloud in result
 * @param {*}
 * @return {*}
 */
void regist_base::GlobalCentroidAlignment() {
	// cloud is empty, throw an error
	try {
		if (this->source_point_cloud_.empty())
			throw "Source point cloud is empty.";
		else if (this->target_point_cloud_.empty())
			throw "Target point cloud is empty.";
	}
	catch (const char* error_message) {
		std::cerr << "Point cloud alignment failed. " << error_message << std::endl;
		return;
	}

	pcl::PointXYZ source_centroid(0.0f, 0.0f, 0.0f);
	pcl::PointXYZ target_centroid(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < this->source_point_cloud_.size(); i++) {
		vvs::operation::PointAddCopy(source_centroid, this->source_point_cloud_[i]);
	}
	for (size_t i = 0; i < this->target_point_cloud_.size(); i++) {
		vvs::operation::PointAddCopy(target_centroid, this->target_point_cloud_[i]);
	}
	vvs::operation::PointDivCopy(source_centroid, this->source_point_cloud_.size());
	vvs::operation::PointDivCopy(target_centroid, this->target_point_cloud_.size());
	// translate point cloud
	pcl::PointXYZ translate = vvs::operation::PointSubAssign(target_centroid, source_centroid);
	vvs::operation::PointCloudAdd(this->result_point_cloud_, translate);

	// record matrix
	vvs::operation::MatrixAddCopy(this->tranformation_matrix_, translate);
}

/***
 * @description: move source cloud to align centroid with target cloud, save the translated cloud in result, using k-nn
 * @param {*}
 * @return {*}
 */
void regist_base::LocalCentroidAlignment() {
	// cloud is empty, throw an error
	try {
		if (this->source_point_cloud_.empty())
			throw "Source point cloud is empty.";
		else if (this->target_point_cloud_.empty())
			throw "Target point cloud is empty.";
	}
	catch (const char* error_message) {
		std::cerr << "Point cloud alignment failed. " << error_message << std::endl;
		return;
	}

	// local alignment, using nearest neighbor search.
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(this->target_point_cloud_.makeShared());

	// calculate align vector
	pcl::PointXYZ align_vector{0.0f, 0.0f, 0.0f};

	// k-nn search
	for (size_t i = 0; i < this->source_point_cloud_.size(); i++) {
		std::vector<int>   index;
		std::vector<float> distance;
		tree.nearestKSearch(this->source_point_cloud_[i], 1, index, distance);
		vvs::operation::PointAddCopy(align_vector, vvs::operation::PointSubAssignColor(this->target_point_cloud_[index[0]], this->source_point_cloud_[i]));
	}
	vvs::operation::PointDivCopy(align_vector, this->source_point_cloud_.size());

	// align and record
	vvs::operation::PointCloudAdd(this->result_point_cloud_, align_vector);
	vvs::operation::MatrixAddCopy(this->tranformation_matrix_, align_vector);
}