/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 20:06:07
 * @LastEditTime: 2022-06-22 18:41:14
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "coder/octree.h"
using namespace vvs::octree;

/***
 * @description: constructor, input kGroupOfFrames
 * @param {int} group_of_frames
 * @return {*}
 */
GOF::GOF(const size_t group_of_frames)
    : kGroupOfFrames{group_of_frames}, frame_patches_{std::vector<pcl::PointCloud<pcl::PointXYZRGB>>(group_of_frames)}, motion_vectors_{std::vector<Eigen::Matrix4f>(group_of_frames)} {
	try {
		if (this->kGroupOfFrames < 1) {
			throw "GOF size is smaller than 1.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "GOF constructing failed : " << error_message << std::endl;
		std::exit(1);
	}
	this->min_x_        = FLT_MAX;
	this->max_x_        = FLT_MIN;
	this->min_y_        = FLT_MAX;
	this->max_y_        = FLT_MIN;
	this->min_z_        = FLT_MAX;
	this->max_z_        = FLT_MIN;
	this->frame_number_ = 0;
}

/***
 * @description: get size of frame_patches_
 * @param {*}
 * @return {size_t} frame_patches_.size()
 */
size_t GOF::size() const {
	try {
		if (this->frame_number_ > 0 && this->frame_number_ <= this->kGroupOfFrames) {
			return this->frame_number_;
		}
		else {
			printf("%lu\n", this->frame_number_);
			throw "Illegal frame number.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "GOF is broken : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get size of frame_patches_[index]
 * @param {size_t} index
 * @return {size_t} frame_patches_[index].size()
 */
size_t GOF::size(const size_t __index) const {
	try {
		if (__index < this->kGroupOfFrames) {
			return this->frame_patches_[__index].size();
		}
		else {
			throw "Out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from function GOF::size(__index) : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get i-th frame_patches_'s j-th element
 * @param {size_t} __x
 * @param {size_t} __y
 * @return {PointXYZRGB&} frame_patches_[__x][__y]
 */
pcl::PointXYZRGB& GOF::operator()(const size_t __x, const size_t __y) {
	try {
		if (__x < this->kGroupOfFrames) {
			if (__y < this->frame_patches_[__x].size()) {
				return this->frame_patches_[__x][__y];
			}
			else {
				throw "Second index out of range.";
			}
		}
		else {
			throw "First index out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from access GOF point : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get i-th frame_patches_'s j-th element
 * @param {size_t} __x
 * @param {size_t} __y
 * @return {PointXYZRGB&} frame_patches_[__x][__y]
 */
const pcl::PointXYZRGB& GOF::operator()(const size_t __x, const size_t __y) const {
	try {
		if (__x < this->kGroupOfFrames) {
			if (__y < this->frame_patches_[__x].size()) {
				return this->frame_patches_[__x][__y];
			}
			else {
				throw "Second index out of range.";
			}
		}
		else {
			throw "First index out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from access GOF point : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: add a patch into gof
 * @param {PointCloud} __patch
 * @param {Matrix4f} __matrix
 * @return {*}
 */
void GOF::AddPatch(const pcl::PointCloud<pcl::PointXYZRGB>& __patch, const Eigen::Matrix4f& __matrix) {
	try {
		// frame_patches_'s size must be less than kGroupOfFrames
		if (this->frame_number_ < this->kGroupOfFrames) {
			// add a patch
			this->frame_patches_[this->frame_number_].resize(__patch.size());
			for (size_t i = 0; i < __patch.size(); i++) {
				this->frame_patches_[this->frame_number_][i] = __patch[i];
				// update range of coordinates
				this->min_x_ = this->min_x_ < __patch[i].x ? this->min_x_ : __patch[i].x;
				this->max_x_ = this->max_x_ > __patch[i].x ? this->max_x_ : __patch[i].x;
				this->min_y_ = this->min_y_ < __patch[i].y ? this->min_y_ : __patch[i].y;
				this->max_y_ = this->max_y_ > __patch[i].y ? this->max_y_ : __patch[i].y;
				this->min_z_ = this->min_z_ < __patch[i].z ? this->min_z_ : __patch[i].z;
				this->max_z_ = this->max_z_ > __patch[i].z ? this->max_z_ : __patch[i].z;
			}
			// add transformation matrix
			if (this->frame_number_ == 0) {
				this->motion_vectors_[this->frame_number_] = Eigen::Matrix4f::Identity();
			}
			else {
				this->motion_vectors_[this->frame_number_] = __matrix;
			}
			this->frame_number_++;
		}
		else {
			throw "Out of GOF size limitation.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Adding patch into GOF failed : " << error_message << std::endl;
	}
}

/***
 * @description: get __index-th patch
 * @param {PointCloud} __patch
 * @param {Matrix4f} __matrix
 * @param {int} __index
 * @return {*}
 */
void GOF::GetPatch(pcl::PointCloud<pcl::PointXYZRGB>& __patch, Eigen::Matrix4f& __matrix, const size_t __index) const {
	try {
		if (__index < this->kGroupOfFrames) {
			__patch.resize(this->frame_patches_[__index].size());
			for (size_t i = 0; i < this->frame_patches_[__index].size(); i++) {
				__patch[i] = this->frame_patches_[__index][i];
			}
			__matrix = this->motion_vectors_[__index];
		}
		else {
			throw "Out of patch range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from function GetPatch() : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get all patches
 * @param {vector<PointCloud>} __patches
 * @param {vector<Matrix4f>} __matrices
 * @return {*}
 */
void GOF::GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& __patches, std::vector<Eigen::Matrix4f>& __matrices) const {
	try {
		if (this->frame_patches_.size() == this->motion_vectors_.size()) {
			__patches.resize(this->frame_patches_.size());
			__matrices.resize(this->motion_vectors_.size());
			for (size_t i = 0; i < this->frame_patches_.size(); i++) {
				__patches[i].resize(this->frame_patches_[i].size());
				for (size_t j = 0; j < this->frame_patches_[i].size(); j++) {
					__patches[i][j] = this->frame_patches_[i][j];
				}
				__matrices[i] = this->motion_vectors_[i];
			}
		}
		else {
			throw "Unmatching size between frames and matrices.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "GOF is broken : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: return maximum span of 3D coordinates
 * @param {*}
 * @return {float}
 */
float GOF::GetResolution() const {
	return std::max(std::max(this->max_x_ - this->min_x_, this->max_y_ - this->min_y_), this->max_z_ - this->min_z_);
}

/***
 * @description: return maximum span of 3D coordinates which is rounded up as a power of 2;
 * @param {*}
 * @return {float}
 */
float GOF::GetResolutionBinary() const {
	return std::pow(2.0f, std::ceil(std::log2(this->GetResolution())));
}

/***
 * @description: return the center of 3D coordinates of patches
 * @param {*}
 * @return {PointXYZ}
 */
pcl::PointXYZ GOF::GetCenter() const {
	return pcl::PointXYZ((this->min_x_ + this->max_x_) / 2, (this->min_y_ + this->max_y_) / 2, (this->min_z_ + this->max_z_) / 2);
}

/***
 * @description: get motion vectors
 * @param {vector<Matrix4f>} __matrices
 * @return {*}
 */
void GOF::GetMatrices(std::vector<Eigen::Matrix4f>& __matrices) const {
	__matrices.resize(this->motion_vectors_.size());
	for (size_t i = 0; i < this->motion_vectors_.size(); i++) {
		__matrices[i] = motion_vectors_[i];
	}
}

/***
 * @description: use k-means to generate a fitting point cloud patch
 * @param {float} kMSEThreshold
 * @param {float} max_correnspondence
 * @param {int} max_iteration
 * @return {*}
 */
void GOF::GenerateFittingPatch(const float kMSEThreshold, const float max_correnspondence, const int max_iteration) {
	// patch coding mode 0-fitting 1-octree
	this->patch_coding_mode_.resize(this->kGroupOfFrames, 0);
	// nicp mse for patches
	this->patches_mses_.resize(this->kGroupOfFrames, 0.0f);

	// do nicp
	for (size_t i = 1; i < this->kGroupOfFrames; i++) {
		vvs::registration::NICP nicp(max_correnspondence, max_iteration);
		nicp.SetSourcePointCloudCopy(this->frame_patches_[i]);
		nicp.SetTargetPointCloudCopy(this->frame_patches_[0]);
		bool converged = nicp.align();
		// not converge
		if (!converged) {
			this->patch_coding_mode_[i] = 1;
			this->patches_mses_[i]      = FLT_MAX;
		}
		else {
			this->patches_mses_[i]   = nicp.GetMSE();
			this->motion_vectors_[i] = nicp.GetMotionVector() * this->motion_vectors_[i];
			nicp.GetResultPointCloudSwap(this->frame_patches_[i]);
			if (this->patches_mses_[i] >= kMSEThreshold) {
				this->patch_coding_mode_[i] = 1;
			}
		}
	}

	// k-means init centers
	size_t point_count = 0, cloud_count = 0, max_index = 0, max_size = 0;

	// k-means whole data and centers
	pcl::PointCloud<pcl::PointXYZRGB> data, centers;
	// add point to data and get mean point cloud size, the largest cloud is recorded in max_index
	for (size_t i = 0; i < this->kGroupOfFrames; i++) {
		if (!this->patch_coding_mode_[i]) {
			point_count += this->frame_patches_[i].size();
			cloud_count++;
			for (auto& k : this->frame_patches_[i]) {
				data.emplace_back(k);
			}
			if (max_size < this->frame_patches_[i].size()) {
				max_index = i, max_size = this->frame_patches_[i].size();
			}
		}
	}

	// if only i-frame can be fitting
	if (cloud_count == 1) {
		this->fitting_patch_.swap(data);
		return;
	}
	else {
		// centers size is mean size of point clouds
		size_t center_size = point_count / cloud_count;
		// use the biggest cloud to be the init centers
		for (auto& k : this->frame_patches_[max_index]) {
			centers.emplace_back(k);
		}
		// down sampling to the center_size, swap random point to end and pop back
		while (centers.size() > center_size) {
			std::swap(centers[rand() % centers.size()], centers[centers.size() - 1]);
			centers.points.pop_back();
		}

		// difference between two iteration
		float last_error = 0.0f;
		for (size_t k = 0; k < max_iteration; k++) {
			// generate search tree
			pcl::search::KdTree<pcl::PointXYZRGB> tree;
			tree.setInputCloud(centers.makeShared());
			// new centers
			std::vector<pcl::PointXYZRGB> result(center_size, pcl::PointXYZRGB());
			// count each cluster's size
			std::vector<int> count(center_size, 0);

			// for each point, search a nearest cluster
			for (auto& i : data) {
				std::vector<int>   idx(1);
				std::vector<float> dis(1);
				tree.nearestKSearch(i, 1, idx, dis);

				// add point to the cluster
				vvs::operation::PointAddCopy(result[idx[0]], i);
				count[idx[0]]++;
			}

			// difference of this and last iteration
			float error = 0.0f;
			for (size_t i = 0; i < center_size; i++) {
				// if this cluster have points
				if (count[i] != 0) {
					// average of a cluster
					vvs::operation::PointDivCopy(result[i], count[i]);
					// calculate the difference
					error += std::pow(result[i].x - centers[i].x, 2) + std::pow(result[i].y - centers[i].y, 2) + std::pow(result[i].z - centers[i].z, 2);
					// update center
					centers[i] = result[i];
				}
			}

			// if difference less than 0.1, end the iterations
			error /= center_size;
			if (std::abs(error - last_error) <= 0.1) {
				break;
			}
			last_error = error;
		}

		// save the result
		this->fitting_patch_.swap(centers);
	}
}

/***
 * @description: cout each patches' mse
 * @param {*}
 * @return {*}
 */
void GOF::OutputPSNR() {
	for (size_t i = 0; i < kGroupOfFrames; i++) {
		printf("Frame #%lu, MSE %.3f \n", i, this->patches_mses_[i]);
		if (!this->patch_coding_mode_[i]) {
			float psnr = vvs::operation::PSNRGeo(this->fitting_patch_, this->frame_patches_[i]);
			printf("GEO PSNR is %.3f\n", psnr);
		}
		else {
			printf("Out of Ths\n");
		}
	}
}

/***
 * @description: output the fitting patch
 * @param {PointCloud<PointXYZRGB>&} __point_cloud
 * @return {*}
 */
void GOF::OutputFittingPatch(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) {
	for (auto& i : this->fitting_patch_) {
		__point_cloud.emplace_back(i);
	}
}