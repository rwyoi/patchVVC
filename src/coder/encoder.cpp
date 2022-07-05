/***
 * @Author: ChenRP07
 * @Date: 2022-06-22 14:55:40
 * @LastEditTime: 2022-07-04 20:33:52
 * @LastEditors: ChenRP07
 * @Description: Implement of Volumetric Video Encoder.
 */
#include "coder/videocoder.h"
using namespace vvs;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

/***
 * @description: constructor for encoder, assign values to all constant variable and init the point_clouds_
 * @param {size_t} __GOF
 * @param {size_t} __patches
 * @param {size_t} __threads
 * @param {float} __mse_ths
 * @param {float} __resolution
 * @return {*}
 */
coder::Encoder::Encoder(const size_t __GOF, const size_t __patches, const size_t __threads, const float __mse_ths, const float __resolution)
    : kGroupOfFrames{__GOF}, kPatchNumber{__patches}, kThreads{__threads}, kMSEThreshold{__mse_ths},
      kMinResolution(__resolution), point_clouds_{std::vector<vvs::octree::GOF>(__patches, vvs::octree::GOF(__GOF))} {
	this->frame_number_ = 0;
	this->i_frame_patches_.resize(this->kPatchNumber);
	this->p_frame_patches_.resize(this->kPatchNumber, std::vector<vvs::type::PFramePatch>(this->kGroupOfFrames - 1));
}

/***
 * @description: add I-frame according the .ply file __file_name
 * @param {string&} __file_name
 * @return {*}
 */
void coder::Encoder::AddIFrame(const std::string& __file_name) {
	try {
		if (this->frame_number_ != 0) {
			throw "Repeatedly adding I-Frame.";
		}
		vvs::segment::SimplePatch         __clustering(this->kPatchNumber);
		pcl::PointCloud<pcl::PointXYZRGB> __i_frame;

		// load point cloud from file
		vvs::io::LoadColorPlyFile(__file_name, __i_frame);

		// add i-frame and clustering
		__clustering.SetPointCloudSwap(__i_frame);
		__clustering.Clustering();

		// get the split results -- patches
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> __i_patches;
		__clustering.GetPatches(__i_patches);
		// add patches into gofs
		for (size_t i = 0; i < this->kPatchNumber; i++) {
			// printf("%lu\n", __i_patches[i].size());
			if (__i_patches[i].empty()) {
				for (size_t j = 0; j < this->kPatchNumber; j++) {
					if (!__i_patches[j].empty()) {
						__i_patches[i].emplace_back(__i_patches[j][0]);
					}
				}
			}
			this->point_clouds_[i].AddPatch(__i_patches[i], Eigen::Matrix4f::Identity());
		}

		this->frame_number_ += 1;
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Encoder : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: add p-frame according to the .ply file __file_name
 * @param {string&} __file_name
 * @return {*}
 */
void coder::Encoder::AddPFrame(const std::string& __file_name) {
	try {
		// check frame counting number
		if (this->frame_number_ == 0) {
			throw "No I-Frame before adding P-Frame.";
		}
		else if (this->frame_number_ == this->kGroupOfFrames) {
			throw "Too many P-Frames.";
		}

		// load p-frame from file
		pcl::PointCloud<pcl::PointXYZRGB> __p_frame;
		vvs::io::LoadColorPlyFile(__file_name, __p_frame);
		std::cout << "Load P-Frame from file " << __file_name << "  ......" << std::endl;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> __i_patches(this->kPatchNumber);
		Eigen::Matrix4f                                temp;

		// extract patches from I-Frame
		for (size_t i = 0; i < this->kPatchNumber; i++) {
			this->point_clouds_[i].GetPatch(__i_patches[i], temp, 0);
		}

		// two-stage parallel icp registration
		vvs::registration::ParallelICP __alignment(this->kThreads, 100.0f, 100);
		__alignment.SetSourcePatchesCopy(__i_patches);
		__alignment.SetTargetPointCloudCopy(__p_frame);

		__alignment.ParallelAlign();

		// get the p-frame patches
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> __p_patches;
		std::vector<Eigen::Matrix4f>                   __p_trans;
		__alignment.GetTargetPatches(__p_patches, __p_trans);

		// add them to point_clouds_
		for (size_t i = 0; i < this->kPatchNumber; i++) {
			this->point_clouds_[i].AddPatch(__p_patches[i], __p_trans[i]);
		}

		// frame count ++
		this->frame_number_ += 1;
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Encoder : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: output __index-th frame to __file_name
 * @param {string&} __file_name
 * @param {size_t&} __index
 * @return {*}
 */
void coder::Encoder::GetFrame(const std::string& __file_name, const size_t& __index) {
	try {
		// check file name is *.ply
		size_t loc = __file_name.find(".ply");

		if (loc == std::string::npos) {
			throw "Wrong file name format.";
		}

		for (size_t i = 0; i < kPatchNumber; i++) {
			// rename the file_name from *.ply to *_patch_i.ply
			std::string __file_format = __file_name;
			__file_format.insert(loc, "_patch_" + std::to_string(i));
			pcl::PointCloud<pcl::PointXYZRGB> temp;
			Eigen::Matrix4f                   trans;
			this->point_clouds_[i].GetPatch(temp, trans, __index);

			vvs::io::SaveColorPlyFile(__file_format, temp);
		}
	}
	catch (const char* error_message) {
		std::cerr << "Cannot output point cloud : " << error_message << std::endl;
		std::exit(1);
	}
}

void coder::Encoder::EncodingProc() {
	while (1) {
		size_t index;
		bool   flag = false;
		this->task_mutex_.lock();
		if (!this->task_queue_.empty()) {
			index = task_queue_.front();
			task_queue_.pop();
			flag = true;
		}
		this->task_mutex_.unlock();
		if (!flag) {
			return;
		}
		else {
			this->point_clouds_[index].GenerateFittingPatch(this->kMSEThreshold, 100.0f, 100);
			this->point_clouds_[index].PatchColorFitting(5);
			this->point_clouds_[index].Output(this->test_[index], 0);
			this->point_clouds_[index].Compression(this->i_frame_patches_[index], this->p_frame_patches_[index]);
		}
	}
}

void coder::Encoder::Encoding() {
	this->test_.resize(this->kPatchNumber);
	this->task_mutex_.lock();
	while (!this->task_queue_.empty()) {
		this->task_queue_.pop();
	}

	for (size_t i = 0; i < this->kPatchNumber; i++) {
		this->task_queue_.push(i);
	}
	this->task_mutex_.unlock();

	std::thread task_threads[this->kThreads];
	for (size_t i = 0; i < this->kThreads; i++) {
		task_threads[i] = std::thread(&vvs::coder::Encoder::EncodingProc, this);
	}

	for (size_t i = 0; i < this->kThreads; i++) {
		task_threads[i].join();
	}
}

void coder::Encoder::Output(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) {
	// for (size_t i = 0; i < this->kPatchNumber; i++) {
	// 	this->point_clouds_[i].OutputPSNR();
	// 	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	// 	this->point_clouds_[i].OutputFittingPatch(cloud);
	// 	for (auto& k : cloud) {
	// 		__point_cloud.emplace_back(k);
	// 	}
	// 	vvs::io::SaveUniqueColorPlyFile("./FitPatch/patch_" + std::to_string(i) + ".ply", cloud);
	// }

	for (auto& i : this->test_) {
		for (auto& j : i) {
			__point_cloud.emplace_back(j);
		}
	}
}

void coder::Encoder::OutputIFrame(const std::string& __i_frame_name) {
	try {
		std::string dir_path = __i_frame_name;
		// if __dir_path is not ended with '/', add a '/'
		if (dir_path.back() != '/') {
			dir_path += '/';
		}
		std::string i_frame_path = dir_path + "IFrame.dat";
		// if this dir do not exist, create it
		if (access(dir_path.c_str(), F_OK) == -1) {
			int flag = mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
			// cannot create, error occurs
			if (flag != 0) {
				std::string err = strerror(errno);
				err             = "Cannot create dir, " + err;
				throw err.c_str();
			}
		}
		// this dir exists
		else {
			// if this iframe exits, remove it
			if (access(i_frame_path.c_str(), F_OK) != -1) {
				int flag_rm = remove(i_frame_path.c_str());
				// cannot remove, error occurs
				if (flag_rm != 0) {
					std::string err = strerror(errno);
					err             = "Cannot remove , " + err;
					throw err.c_str();
				}
			}
		}

		// open this file
		FILE* fp = fopen(i_frame_path.c_str(), "w");
		if (fp == nullptr) {
			std::string err = strerror(errno);
			err             = "Cannot open IFrame data file, " + err;
			fclose(fp);
			throw err.c_str();
		}

		// write constant variable, GOF, Patch and MinResolution
		fwrite(&(this->kGroupOfFrames), sizeof(size_t), 1, fp);
		fwrite(&(this->kPatchNumber), sizeof(size_t), 1, fp);
		fwrite(&(this->kMinResolution), sizeof(float), 1, fp);

		// write each patch
		for (size_t i = 0; i < this->kPatchNumber; i++) {
			fwrite(&(this->i_frame_patches_[i].center_), sizeof(pcl::PointXYZ), 1, fp);
			fwrite(&(this->i_frame_patches_[i].tree_height_), sizeof(size_t), 1, fp);

			size_t data_size;
			data_size = this->i_frame_patches_[i].octree_.size();
			fwrite(&data_size, sizeof(size_t), 1, fp);
			fwrite(this->i_frame_patches_[i].octree_.c_str(), sizeof(char), this->i_frame_patches_[i].octree_.size(), fp);

			fwrite(&(this->i_frame_patches_[i].block_number_), sizeof(size_t), 1, fp);

			data_size = this->i_frame_patches_[i].colors_.size();
			fwrite(&data_size, sizeof(size_t), 1, fp);
			fwrite(this->i_frame_patches_[i].colors_.c_str(), sizeof(char), this->i_frame_patches_[i].colors_.size(), fp);
		}

		fclose(fp);
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Encoder OutputIFrame : " << error_message << std::endl;
		std::exit(1);
	}
}

void coder::Encoder::OutputPFrame(const std::string& __p_frame_name) {
	try {
		std::string dir_path = __p_frame_name;
		// if __dir_path is not ended with '/', add a '/'
		if (dir_path.back() != '/') {
			dir_path += '/';
		}

		// if this dir do not exist, create it
		if (access(dir_path.c_str(), F_OK) == -1) {
			int flag = mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
			// cannot create, error occurs
			if (flag != 0) {
				std::string err = strerror(errno);
				err             = "Cannot create dir, " + err;
				throw err.c_str();
			}
		}
		// this dir exists
		for (size_t i = 0; i < kGroupOfFrames - 1; i++) {
			std::string p_frame_path = dir_path + "PFrame" + std::to_string(i) + ".dat";
			// if this pframe exits, remove it
			if (access(p_frame_path.c_str(), F_OK) != -1) {
				int flag_rm = remove(p_frame_path.c_str());
				// cannot remove, error occurs
				if (flag_rm != 0) {
					std::string err = strerror(errno);
					err             = "Cannot remove , " + err;
					throw err.c_str();
				}
			}

			// open this file
			FILE* fp = fopen(p_frame_path.c_str(), "w");
			if (fp == nullptr) {
				std::string err = strerror(errno);
				err             = "Cannot open PFrame data file, " + err;
				fclose(fp);
				throw err.c_str();
			}

			// write each patch
			for (size_t j = 0; j < this->kPatchNumber; j++) {
				fwrite(&(this->p_frame_patches_[j][i].is_independent_), sizeof(bool), 1, fp);
				size_t data_size;
				if (this->p_frame_patches_[j][i].is_independent_) {
					fwrite(&(this->p_frame_patches_[j][i].center_), sizeof(pcl::PointXYZ), 1, fp);
					fwrite(&(this->p_frame_patches_[j][i].tree_height_), sizeof(size_t), 1, fp);

					data_size = this->p_frame_patches_[j][i].octree_.size();
					fwrite(&data_size, sizeof(size_t), 1, fp);
					fwrite(this->p_frame_patches_[j][i].octree_.c_str(), sizeof(char), this->p_frame_patches_[j][i].octree_.size(), fp);
				}

				fwrite(&(this->p_frame_patches_[j][i].block_number_), sizeof(size_t), 1, fp);

				data_size = this->p_frame_patches_[j][i].colors_.size();
				fwrite(&data_size, sizeof(size_t), 1, fp);
				fwrite(this->p_frame_patches_[j][i].colors_.c_str(), sizeof(char), this->p_frame_patches_[j][i].colors_.size(), fp);

				fwrite(&(this->p_frame_patches_[j][i].motion_vector_), sizeof(Eigen::Matrix4f), 1, fp);
			}

			fclose(fp);
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Encoder OutputIFrame : " << error_message << std::endl;
		std::exit(1);
	}
}
#pragma GCC diagnostic pop