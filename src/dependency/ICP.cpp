/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 17:00:23
 * @LastEditTime: 2022-07-01 10:39:59
 * @LastEditors: ChenRP07
 * @Description: Implement of point cloud registration algorithm, including [1], [2]
 */
#include "dependency/registration.h"
using namespace vvs::registration;

/***
 * @description: constructor
 * @param {float} max_correspondence
 * @param {int} max_iteration
 * @param {float} max_mse_difference
 * @param {float} max_tranformation_difference
 * @return {*}
 */
ICP::ICP(const float max_correspondence, const int max_iteration, const float max_mse_difference, const float max_tranformation_diference)
    : kCorrespondenceThreshold{max_correspondence}, kIterationThreshold{max_iteration}, kMSEThreshold{max_mse_difference}, kTransformationThreshold{max_tranformation_diference}, regist_base() {}

/***
 * @description: do Iterative Closest Point Algorithm, using centroid alignment
 * @param {int} centroid_alignment_type, 0 none, 1 global, 2 local.
 * @return {bool} Algorithm converged ?
 */
bool ICP::align(const int centroid_alignment_type) {
	// cloud is empty or wrong centroid alignment type, throw an error
	try {
		if (this->source_point_cloud_.empty())
			throw "Source point cloud is empty.";
		else if (this->target_point_cloud_.empty())
			throw "Target point cloud is empty.";

		if (centroid_alignment_type != 0 && centroid_alignment_type != 1 && centroid_alignment_type != 2)
			throw "Wrong centroid alignment type";
	}
	catch (const char* error_message) {
		std::cerr << "Point cloud alignment failed. " << error_message << std::endl;
		return 0;
	}

	if (centroid_alignment_type == 1) {
		this->GlobalCentroidAlignment();
	}
	else if (centroid_alignment_type == 2) {
		this->LocalCentroidAlignment();
	}

	// Iterative Closest Point Algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

	// set point cloud
	icp.setInputSource(this->result_point_cloud_.makeShared());
	icp.setInputTarget(this->target_point_cloud_.makeShared());

	// set manaully parameter
	icp.setMaxCorrespondenceDistance(this->kCorrespondenceThreshold);
	icp.setTransformationEpsilon(this->kTransformationThreshold);
	icp.setEuclideanFitnessEpsilon(this->kMSEThreshold);
	icp.setMaximumIterations(this->kIterationThreshold);

	pcl::PointCloud<pcl::PointXYZRGB> temp_point_cloud;

	// do alignment
	icp.align(temp_point_cloud);

	// check whether converged
	try {
		if (icp.hasConverged() == false) {
			throw "Cannot converge.";
		}
		else
			this->result_point_cloud_.swap(temp_point_cloud);
	}
	catch (const char* error_message) {
		std::cerr << "Alignment failed. " << error_message << " Source size " << this->GetSourcePointCloudSize() << ". Target size " << this->GetTargetPointCloudSize() << "." << std::endl;
		return 0;
	}

	// record result
	this->mean_squred_error_    = icp.getFitnessScore();
	this->tranformation_matrix_ = icp.getFinalTransformation() * this->tranformation_matrix_;

	return 1;
}

/***
 * @description: constructor
 * @param {float} max_correspondence
 * @param {int} max_iteration
 * @param {float} normal_search_radius
 * @param {float} max_mse_difference
 * @param {float} max_tranformation_diference
 * @return {*}
 */
NICP::NICP(const float max_correspondence, const int max_iteration, const float normal_search_radius, const float max_mse_difference, const float max_tranformation_diference)
    : kCorrespondenceThreshold{max_correspondence}, kIterationThreshold{max_iteration},
      kNormalRadius(normal_search_radius), kMSEThreshold{max_mse_difference}, kTransformationThreshold{max_tranformation_diference}, regist_base() {
	this->normaled_result_point_cloud_.reserve(10);
	this->normaled_target_point_cloud_.reserve(10);
}

/***
 * @description: compute normals
 * @param {*}
 * @return {*}
 */
void NICP::ComputeNormals() {
	// cloud is empty or wrong centroid alignment type, throw an error
	try {
		if (this->source_point_cloud_.empty())
			throw "Source point cloud is empty.";
		else if (this->target_point_cloud_.empty())
			throw "Target point cloud is empty.";
	}
	catch (const char* error_message) {
		std::cerr << "Computing point cloud normals failed. " << error_message << std::endl;
		return;
	}

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> result_estimator, target_estimator;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr           result_kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>), target_kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>                         result_normals, target_normals;

	result_estimator.setInputCloud(this->result_point_cloud_.makeShared());
	result_estimator.setRadiusSearch(this->kNormalRadius);
	result_estimator.setSearchMethod(result_kdtree);
	result_estimator.compute(result_normals);

	target_estimator.setInputCloud(this->target_point_cloud_.makeShared());
	target_estimator.setRadiusSearch(this->kNormalRadius);
	target_estimator.setSearchMethod(target_kdtree);
	target_estimator.compute(target_normals);

	this->normaled_result_point_cloud_.resize(this->result_point_cloud_.size());
	this->normaled_target_point_cloud_.resize(this->target_point_cloud_.size());

	for (size_t i = 0; i < this->result_point_cloud_.size(); i++) {
		this->normaled_result_point_cloud_[i].x         = this->result_point_cloud_[i].x;
		this->normaled_result_point_cloud_[i].y         = this->result_point_cloud_[i].y;
		this->normaled_result_point_cloud_[i].z         = this->result_point_cloud_[i].z;
		this->normaled_result_point_cloud_[i].rgb       = this->result_point_cloud_[i].rgb;
		this->normaled_result_point_cloud_[i].normal_x  = result_normals[i].normal_x;
		this->normaled_result_point_cloud_[i].normal_y  = result_normals[i].normal_y;
		this->normaled_result_point_cloud_[i].normal_z  = result_normals[i].normal_z;
		this->normaled_target_point_cloud_[i].curvature = result_normals[i].curvature;
	}

	for (size_t i = 0; i < this->target_point_cloud_.size(); i++) {
		this->normaled_target_point_cloud_[i].x         = this->target_point_cloud_[i].x;
		this->normaled_target_point_cloud_[i].y         = this->target_point_cloud_[i].y;
		this->normaled_target_point_cloud_[i].z         = this->target_point_cloud_[i].z;
		this->normaled_target_point_cloud_[i].rgb       = this->target_point_cloud_[i].rgb;
		this->normaled_target_point_cloud_[i].normal_x  = target_normals[i].normal_x;
		this->normaled_target_point_cloud_[i].normal_y  = target_normals[i].normal_y;
		this->normaled_target_point_cloud_[i].normal_z  = target_normals[i].normal_z;
		this->normaled_target_point_cloud_[i].curvature = target_normals[i].curvature;
	}
}

/***
 * @description: do normal icp, using centroid alignment
 * @param {int} centroid_alignment_type, 0 none, 1 global, 2 local
 * @return {*}
 */
bool NICP::align(const int centroid_alignment_type) {
	// cloud is empty or wrong centroid alignment type, throw an error
	try {
		if (this->source_point_cloud_.empty())
			throw "Source point cloud is empty.";
		else if (this->target_point_cloud_.empty())
			throw "Target point cloud is empty.";

		if (centroid_alignment_type != 0 && centroid_alignment_type != 1 && centroid_alignment_type != 2)
			throw "Wrong centroid alignment type";
	}
	catch (const char* error_message) {
		std::cerr << "Point cloud alignment failed. " << error_message << std::endl;
		return 0;
	}

	// do centroid alignment
	if (centroid_alignment_type == 1) {
		this->GlobalCentroidAlignment();
	}
	else if (centroid_alignment_type == 2) {
		this->LocalCentroidAlignment();
	}

	// compute normals
	this->ComputeNormals();

	// do nicp
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> nicp;
	nicp.setInputSource(this->normaled_result_point_cloud_.makeShared());
	nicp.setInputTarget(this->normaled_target_point_cloud_.makeShared());

	nicp.setMaxCorrespondenceDistance(this->kCorrespondenceThreshold);
	nicp.setEuclideanFitnessEpsilon(this->kMSEThreshold);
	nicp.setTransformationEpsilon(this->kTransformationThreshold);
	nicp.setMaximumIterations(this->kIterationThreshold);

	pcl::PointCloud<pcl::PointXYZRGBNormal> temp_point_cloud;
	nicp.align(temp_point_cloud);

	// check whether converged
	try {
		// not, throw an error
		if (nicp.hasConverged() == false) {
			throw "Cannot converge.";
		}
		// converged, save result
		else {
			for (size_t i = 0; i < temp_point_cloud.size(); i++) {
				this->result_point_cloud_[i].x   = temp_point_cloud[i].x;
				this->result_point_cloud_[i].y   = temp_point_cloud[i].y;
				this->result_point_cloud_[i].z   = temp_point_cloud[i].z;
				this->result_point_cloud_[i].rgb = temp_point_cloud[i].rgb;
			}
		}
	}
	catch (const char* error_message) {
		std::cerr << "Alignment failed. " << error_message << " Source size " << this->GetSourcePointCloudSize() << ". Target size " << this->GetTargetPointCloudSize() << "." << std::endl;
		return 0;
	}

	// save tranformation_matrix_
	this->mean_squred_error_    = nicp.getFitnessScore();
	this->tranformation_matrix_ = nicp.getFinalTransformation() * this->tranformation_matrix_;
	return 1;
}

/***
 * @description: constructor
 * @param {int} thread_num
 * @param {float} max_correspondence
 * @param {int} max_iteration
 * @param {float} max_mse_difference
 * @param {float} max_tranformation_diference
 * @return {*}
 */
ParallelICP::ParallelICP(const int thread_num, const float max_correspondence, const int max_iteration, const float max_mse_difference, const float max_tranformation_diference)
    : kThreads(thread_num), kCorrespondenceThreshold{max_correspondence}, kIterationThreshold{max_iteration}, kMSEThreshold{max_mse_difference}, kTransformationThreshold{max_tranformation_diference} {
}

/***
 * @description: set source point cloud patches, using swap
 * @param {vector<PointCloud>&} patches
 * @return {*}
 */
void ParallelICP::SetSourcePatchesSwap(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& patches) {
	// resize containers
	this->source_patches_.resize(patches.size());
	this->motion_vectors_.resize(patches.size());
	this->mean_squred_errors_.resize(patches.size());

	// swap patches to source_patches_
	for (size_t i = 0; i < patches.size(); i++) {
		this->source_patches_[i].swap(patches[i]);
	}
}

/***
 * @description: set source point cloud patch copy
 * @param {vector<PointCloud>&} patches
 * @return {*}
 */
void ParallelICP::SetSourcePatchesCopy(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& patches) {
	// resize containers
	this->source_patches_.resize(patches.size());
	this->motion_vectors_.resize(patches.size());
	this->mean_squred_errors_.resize(patches.size());

	// copy patches to source_patches_
	for (size_t i = 0; i < patches.size(); i++) {
		for (size_t j = 0; j < patches[i].size(); j++) {
			this->source_patches_[i].push_back(patches[i][j]);
		}
	}
}

/***
 * @description: set target point cloud, using copy
 * @param {PointCloud&} point_cloud
 * @return {*}
 */
void ParallelICP::SetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	// resize containers
	this->target_point_cloud_.resize(point_cloud.size());
	// copy point cloud
	for (size_t i = 0; i < point_cloud.size(); i++) {
		this->target_point_cloud_[i] = point_cloud[i];
	}
}

/***
 * @description: set target point cloud, using swap
 * @param {PointCloud&} point_cloud
 * @return {*}
 */
void ParallelICP::SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	// swap point cloud
	this->target_point_cloud_.swap(point_cloud);
}

/***
 * @description: thread proccess function
 * @param {*}
 * @return {*}
 */
void ParallelICP::ThreadProcess() {
	while (true) {
		// get task from pool
		int task_index = -1;
		this->task_pool_mutex_.lock();
		if (!this->task_pool_.empty()) {
			task_index = this->task_pool_.front();
			this->task_pool_.pop();
		}
		this->task_pool_mutex_.unlock();
		// all task has done
		if (task_index == -1)
			return;
		else {
			// do icp
			try {
				ICP task(this->kCorrespondenceThreshold, this->kIterationThreshold, this->kMSEThreshold, this->kTransformationThreshold);
				task.SetSourcePointCloudCopy(this->source_patches_[task_index]);
				task.SetTargetPointCloudCopy(this->target_point_cloud_);
				bool converge                         = task.align(2);
				this->motion_vectors_[task_index]     = task.GetMotionVector();
				this->mean_squred_errors_[task_index] = task.GetMSE();
				if (converge == false)
					throw "ICP is not converged";
			}
			catch (const char* error_message) {
				IO_mutex_.lock();
				std::cerr << "Patch #" << task_index << " registration error : " << error_message << std::endl;
				IO_mutex_.unlock();
			}
		}
	}
}

/***
 * @description: parallel local icp registration
 * @param {*}
 * @return {*}
 */
void ParallelICP::ParallelAlign() {
	// global icp
	// collect whole i-frame
	pcl::PointCloud<pcl::PointXYZRGB> __i_frame;
	for (size_t i = 0; i < this->source_patches_.size(); i++) {
		for (size_t j = 0; j < this->source_patches_[i].size(); j++) {
			__i_frame.emplace_back(this->source_patches_[i][j]);
		}
	}
	size_t i_size = __i_frame.size();

	// do icp
	ICP __global(this->kCorrespondenceThreshold, this->kIterationThreshold, this->kMSEThreshold, this->kTransformationThreshold);
	__global.SetTargetPointCloudSwap(__i_frame);
	__global.SetSourcePointCloudSwap(this->target_point_cloud_);

	bool converge        = __global.align(1);
	this->global_vector_ = __global.GetMotionVector();
	float error          = __global.GetMSE();

	// get result
	__global.GetResultPointCloudSwap(this->target_point_cloud_);
	std::cout << "Global alignment";
	if (!converge) {
		std::cout << " not";
	}
	std::cout << " converged, MSE is " << error << ", Target size is " << i_size << ", Source size is " << this->target_point_cloud_.size() << std::endl;

	// add task to pool
	for (size_t i = 0; i < this->source_patches_.size(); i++) {
		this->task_pool_.push(i);
	}

	// kThreads thread
	std::thread ICP_threads[kThreads];
	// create threads
	for (int i = 0; i < kThreads; i++) {
		ICP_threads[i] = std::thread(&ParallelICP::ThreadProcess, this);
	}
	// wait for all threads completing
	for (auto& th : ICP_threads) {
		th.join();
	}

	// information of mse, avg-dev-min-max
	float MSE_sum   = std::accumulate(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end(), 0);
	float MSE_avg   = MSE_sum / this->mean_squred_errors_.size();
	float MSE_accum = 0.0;
	std::for_each(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end(), [&](const float d) { MSE_accum += std::pow(d - MSE_avg, 2); });
	float MSE_dev = std::sqrt(MSE_accum / (this->mean_squred_errors_.size() - 1));

	std::cout << "Local alignment complete, using " << this->kThreads << " threads." << std::endl;
	std::cout << "MSE of " << this->mean_squred_errors_.size() << " patches : ";
	std::cout << "avg is " << MSE_avg << ", dev is " << MSE_dev << ", min is " << *std::min_element(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end()) << ", max is "
	          << *std::max_element(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end()) << std::endl;
}

/***
 * @description: split targe point cloud into patches according to the nearest neighbor in source patches,
                 transform the split patches and record matrices and patches.
 * @param {vector<PointCloud>&} patches
 * @param {vector<Matrix4f>&} matrices
 * @return {*}
 */
void ParallelICP::GetTargetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& patches, std::vector<Eigen::Matrix4f>& matrices) {
	try {
		// target patches are split already, output directly
		if (indexed && (this->target_patch_index_.size() == this->target_point_cloud_.size())) {
			patches.resize(this->source_patches_.size());
			matrices.resize(this->motion_vectors_.size());
			for (size_t i = 0; i < this->target_patch_index_.size(); i++) {
				patches[this->target_patch_index_[i]].push_back(this->target_point_cloud_[i]);
			}

			// tranform the patch and record the matrix, make the target patch align to the related source patch
			for (size_t i = 0; i < this->motion_vectors_.size(); i++) {
				vvs::operation::PointCloudMul(patches[i], this->motion_vectors_[i].inverse());
				matrices[i] = this->motion_vectors_[i].inverse() * this->global_vector_;
			}
		}
		// target patches are not split already, split them
		else if (!indexed) {
			// transform the source patches
			pcl::PointCloud<pcl::PointXYZRGB> transformed_source;
			std::vector<size_t>               source_index;
			if (this->source_patches_.size() != this->motion_vectors_.size()) {
				throw "Patch size and Matrix size don't match.";
			}
			for (size_t i = 0; i < this->source_patches_.size(); i++) {
				vvs::operation::PointCloudMulAdd(transformed_source, this->source_patches_[i], this->motion_vectors_[i]);
				for (size_t j = 0; j < this->source_patches_[i].size(); j++) {
					source_index.emplace_back(i);
				}
			}
			// form a kdtree and search nearest neighbor for target points
			pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
			kdtree.setInputCloud(transformed_source.makeShared());

			this->target_patch_index_.resize(this->target_point_cloud_.size());

			for (size_t i = 0; i < this->target_point_cloud_.size(); i++) {
				// do 1-nn search
				std::vector<int>   __index(1);
				std::vector<float> __distance(1);
				kdtree.nearestKSearch(this->target_point_cloud_[i], 1, __index, __distance);

				// add it into related patch
				this->target_patch_index_[i] = source_index[__index[0]];
			}

			// split
			patches.resize(this->source_patches_.size());
			matrices.resize(this->motion_vectors_.size());

			for (size_t i = 0; i < this->target_patch_index_.size(); i++) {
				patches[this->target_patch_index_[i]].push_back(this->target_point_cloud_[i]);
			}

			// tranform the patch and record the matrix, make the target patch align to the related source patch
			for (size_t i = 0; i < this->motion_vectors_.size(); i++) {
				vvs::operation::PointCloudMul(patches[i], this->motion_vectors_[i].inverse());
				matrices[i] = this->motion_vectors_[i].inverse() * this->global_vector_;
			}

			this->indexed = true;
		}
		else {
			throw "Target patches split error.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "ParallelICP failed : " << error_message << std::endl;
	}
}

/***
 * @description: get patches mses
 * @param {vector<float>&} mses
 * @return {*}
 */
void ParallelICP::GetTargetMSEs(std::vector<float>& mses) const {
	try {
		if (this->mean_squred_errors_.size() != this->source_patches_.size())
			throw "Target patches mse not match.";
		else {
			mses.resize(this->mean_squred_errors_.size());
			for (size_t i = 0; i < this->mean_squred_errors_.size(); i++) {
				mses[i] = this->mean_squred_errors_[i];
			}
		}
	}
	catch (const char* error_message) {
		std::cerr << "ParallelICP failed : " << error_message << std::endl;
	}
}

// [1] P. Besl and N. McKay, "A method for registration of 3-D shapes", IEEE Transcation on Pattern Anal Mach. IntellTransactions on Pattern Analysis and Machine Intelligence 1992
// [2] Serafin, Jacopo and Grisetti, Giorgio "NICP: Dense normal based point cloud registration" IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2015