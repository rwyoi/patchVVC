/***
 * @Author: ChenRP07
 * @Date: 2022-07-01 16:21:06
 * @LastEditTime: 2022-07-04 19:33:06
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "coder/videocoder.h"
using namespace vvs;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

/***
 * @description: constructor, init the variable
 * @param {*}
 * @return {*}
 */
coder::Decoder::Decoder(const size_t __ths) : kThreads{__ths} {
	this->frame_number_  = 0;
	this->kGroupOfFrame  = 0;
	this->kPatchNumber   = 0;
	this->kMinResolution = 0.0f;
}

/***
 * @description: Read IFrame data from file.
 * @param {string&} __i_frame_name
 * @return {*}
 */
void coder::Decoder::AddIFrame(const std::string& __i_frame_name) {
	try {
		// dir_path must end with '/'
		std::string dir_path = __i_frame_name;
		if (dir_path.back() != '/') {
			dir_path += '/';
		}

		// if dir_path/IFrame.dat exist?
		std::string i_frame_path = dir_path + "IFrame.dat";
		if (access(dir_path.c_str(), F_OK) == -1) {
			throw "No such IFrame directory.";
		}
		else {
			if (access(i_frame_path.c_str(), F_OK) == -1) {
				throw "No such IFrame file.";
			}
		}
		// open file
		FILE* fp = fopen(i_frame_path.c_str(), "r");
		if (fp == nullptr) {
			std::string err = strerror(errno);
			err             = "Cannot open IFrame data file, " + err;
			fclose(fp);
			throw err.c_str();
		}

		// const variables are already valued?
		if (this->kGroupOfFrame != 0 || this->kPatchNumber != 0 || this->kMinResolution != 0.0f) {
			throw "IFrame already exist.";
		}

		// read these variables
		fread(&(this->kGroupOfFrame), sizeof(size_t), 1, fp);
		fread(&(this->kPatchNumber), sizeof(size_t), 1, fp);
		fread(&(this->kMinResolution), sizeof(float), 1, fp);

		this->I_Frame_Patches_.resize(this->kPatchNumber), this->P_Frame_Patches_.resize(this->kPatchNumber);
		// for each patch
		for (size_t i = 0; i < this->kPatchNumber; i++) {
			// read the fitting patch octree
			fread(&(this->I_Frame_Patches_[i].center_), sizeof(pcl::PointXYZ), 1, fp);
			fread(&(this->I_Frame_Patches_[i].tree_height_), sizeof(size_t), 1, fp);

			size_t data_size;
			fread(&data_size, sizeof(size_t), 1, fp);
			this->I_Frame_Patches_[i].octree_.resize(data_size);
			for (size_t j = 0; j < data_size; j++) {
				fread(&(this->I_Frame_Patches_[i].octree_[j]), sizeof(char), 1, fp);
			}

			// read color block number
			fread(&(this->I_Frame_Patches_[i].block_number_), sizeof(size_t), 1, fp);

			// read colors
			fread(&data_size, sizeof(size_t), 1, fp);
			this->I_Frame_Patches_[i].colors_.resize(data_size);
			for (size_t j = 0; j < data_size; j++) {
				fread(&(this->I_Frame_Patches_[i].colors_[j]), sizeof(char), 1, fp);
			}
		}
		fclose(fp);
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Decoder GetIFrame " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description:
 * @param {string&} __p_frame_name
 * @return {*}
 */
void coder::Decoder::AddPFrame(const std::string& __p_frame_name) {
	try {
		// all PFrames are encoded?
		if (this->frame_number_ >= (this->kGroupOfFrame - 1)) {
			throw "All PFrames are already encoded.";
		}

		// dir_path must end with '/'
		std::string dir_path = __p_frame_name;
		if (dir_path.back() != '/') {
			dir_path += '/';
		}

		// if data file exists?
		std::string p_frame_path = dir_path + "PFrame" + std::to_string(this->frame_number_) + ".dat";
		if (access(dir_path.c_str(), F_OK) == -1) {
			throw "No such IFrame directory.";
		}
		else {
			if (access(p_frame_path.c_str(), F_OK) == -1) {
				throw "No such IFrame file.";
			}
		}

		// open file
		FILE* fp = fopen(p_frame_path.c_str(), "r");
		if (fp == nullptr) {
			std::string err = strerror(errno);
			err             = "Cannot open IFrame data file, " + err;
			fclose(fp);
			throw err.c_str();
		}

		// read data
		for (size_t i = 0; i < this->kPatchNumber; i++) {
			// clear the container
			this->P_Frame_Patches_[i].clear();

			// read coding tag
			fread(&(this->P_Frame_Patches_[i].is_independent_), sizeof(bool), 1, fp);

			// string size
			size_t data_size;
			// if independent coding, read octree
			if (this->P_Frame_Patches_[i].is_independent_) {
				fread(&(this->P_Frame_Patches_[i].center_), sizeof(pcl::PointXYZ), 1, fp);
				fread(&(this->P_Frame_Patches_[i].tree_height_), sizeof(size_t), 1, fp);

				fread(&data_size, sizeof(size_t), 1, fp);
				this->P_Frame_Patches_[i].colors_.resize(data_size);
				for (size_t j = 0; j < data_size; j++) {
					fread(&(this->P_Frame_Patches_[i].colors_[j]), sizeof(char), 1, fp);
				}
			}

			// read color block numbers
			fread(&(this->P_Frame_Patches_[i].block_number_), sizeof(size_t), 1, fp);

			// read colors
			fread(&data_size, sizeof(size_t), 1, fp);
			this->P_Frame_Patches_[i].colors_.resize(data_size);
			for (size_t j = 0; j < data_size; j++) {
				fread(&(this->P_Frame_Patches_[i].colors_[j]), sizeof(char), 1, fp);
			}

			// read motion vector
			fread(&(this->P_Frame_Patches_[i].motion_vector_), sizeof(Eigen::Matrix4f), 1, fp);
		}
		fclose(fp);
		// add frame number
		this->frame_number_++;
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Decoder GetIFrame " << error_message << std::endl;
		std::exit(1);
	}
}

void coder::Decoder::GetIFrame(pcl::PointCloud<pcl::PointXYZRGB>& __i_frame) {
	timeval time1, time2;
	gettimeofday(&time1, nullptr);
	this->task_mutex_.lock();
	while (!task_pool_.empty()) {
		task_pool_.pop();
	}
	for (size_t i = 0; i < this->kPatchNumber; i++) {
		task_pool_.push(i);
	}
	this->task_mutex_.unlock();

	this->fitting_patches_.clear(), this->fitting_colors_.clear();
	this->fitting_patches_.resize(this->kPatchNumber);
	this->fitting_colors_.resize(this->kPatchNumber);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> IPatches(this->kPatchNumber);
	std::thread                                    task_threads[this->kThreads];
	for (auto& i : task_threads) {
		i = std::thread(&coder::Decoder::GetIFrameProc, this);
	}
	for (auto& i : task_threads) {
		i.join();
	}

	for (size_t i = 0; i < kPatchNumber; i++) {
		for (size_t j = 0; j < this->fitting_patches_[i].size(); j++) {
			pcl::PointXYZRGB point;
			point.x = this->fitting_patches_[i][j].x, point.y = this->fitting_patches_[i][j].y, point.z = this->fitting_patches_[i][j].z;
			point.r = static_cast<uint8_t>(this->fitting_colors_[i][j].r_), point.g = static_cast<uint8_t>(this->fitting_colors_[i][j].g_),
			point.b = static_cast<uint8_t>(this->fitting_colors_[i][j].b_);
			IPatches[i].emplace_back(point);
		}
	}
	gettimeofday(&time2, nullptr);
	printf("IFrame decoding cost %.6fs.\n", time2.tv_sec - time1.tv_sec + (float)(time2.tv_usec - time1.tv_usec) / 1e6);

	for (auto& i : IPatches) {
		for (auto& j : i) {
			__i_frame.emplace_back(j);
		}
	}
}

void coder::Decoder::GetPFrame(pcl::PointCloud<pcl::PointXYZRGB>& __p_frame) {
	timeval time1, time2;
	gettimeofday(&time1, nullptr);
	this->task_mutex_.lock();
	while (!task_pool_.empty()) {
		task_pool_.pop();
	}
	for (size_t i = 0; i < this->kPatchNumber; i++) {
		task_pool_.push(i);
	}
	this->task_mutex_.unlock();

	this->single_patches_.clear();
	this->p_colors_.clear();
	this->single_patches_.resize(this->kPatchNumber);
	this->p_colors_.resize(this->kPatchNumber);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> Patches(this->kPatchNumber);
	std::thread                                    task_threads[this->kThreads];
	for (auto& i : task_threads) {
		i = std::thread(&coder::Decoder::GetPFrameProc, this);
	}
	for (auto& i : task_threads) {
		i.join();
	}

	for (size_t i = 0; i < kPatchNumber; i++) {
		if (!this->P_Frame_Patches_[i].is_independent_) {
			for (size_t j = 0; j < this->fitting_patches_[i].size(); j++) {
				pcl::PointXYZRGB point;
				point.x = this->fitting_patches_[i][j].x, point.y = this->fitting_patches_[i][j].y, point.z = this->fitting_patches_[i][j].z;
				point.r = static_cast<uint8_t>(this->p_colors_[i][j].r_), point.g = static_cast<uint8_t>(this->p_colors_[i][j].g_), point.b = static_cast<uint8_t>(this->p_colors_[i][j].b_);
				Patches[i].emplace_back(point);
			}
		}
		else {
			for (size_t j = 0; j < this->single_patches_[i].size(); j++) {
				pcl::PointXYZRGB point;
				point.x = this->single_patches_[i][j].x, point.y = this->single_patches_[i][j].y, point.z = this->single_patches_[i][j].z;
				point.r = static_cast<uint8_t>(this->p_colors_[i][j].r_), point.g = static_cast<uint8_t>(this->p_colors_[i][j].g_), point.b = static_cast<uint8_t>(this->p_colors_[i][j].b_);
				Patches[i].emplace_back(point);
			}
		}
	}
	gettimeofday(&time2, nullptr);
	printf("PFrame decoding cost %.6fs.\n", time2.tv_sec - time1.tv_sec + (float)(time2.tv_usec - time1.tv_usec) / 1e6);

	for (size_t i = 0; i < this->kPatchNumber; i++) {
		vvs::operation::PointCloudMul(Patches[i], this->P_Frame_Patches_[i].motion_vector_.inverse());
		for (auto& j : Patches[i]) {
			__p_frame.emplace_back(j);
		}
	}
}

void coder::Decoder::GetIFrameProc() {
	while (true) {
		size_t index;
		bool   flag = false;
		this->task_mutex_.lock();
		if (!this->task_pool_.empty()) {
			index = this->task_pool_.front();
			this->task_pool_.pop();
			flag = true;
		}
		this->task_mutex_.unlock();
		if (!flag) {
			return;
		}
		else {
			std::thread color_proc;
			color_proc = std::thread(&coder::Decoder::GetColorProc, this, index, 0);
			vvs::octree::DeOctree3D deoctree(this->kMinResolution);
			deoctree.SetCenter(this->I_Frame_Patches_[index].center_);
			deoctree.SetHeight(this->I_Frame_Patches_[index].tree_height_);
			deoctree.SetTree(this->I_Frame_Patches_[index].octree_);
			deoctree.GetPatch(this->fitting_patches_[index]);
			color_proc.join();
		}
	}
}

void coder::Decoder::GetPFrameProc() {
	while (true) {
		size_t index;
		bool   flag = false;
		this->task_mutex_.lock();
		if (!this->task_pool_.empty()) {
			index = this->task_pool_.front();
			this->task_pool_.pop();
			flag = true;
		}
		this->task_mutex_.unlock();
		if (!flag) {
			return;
		}
		else {
			if (!this->P_Frame_Patches_[index].is_independent_) {
				GetColorProc(index, 1);
			}
			else {
				std::thread color_proc;
				color_proc = std::thread(&coder::Decoder::GetColorProc, this, index, 2);

				vvs::octree::DeOctree3D deoctree(this->kMinResolution);
				deoctree.SetCenter(this->P_Frame_Patches_[index].center_);
				deoctree.SetHeight(this->P_Frame_Patches_[index].tree_height_);
				deoctree.SetTree(this->P_Frame_Patches_[index].octree_);
				deoctree.GetPatch(this->single_patches_[index]);
				color_proc.join();
			}
		}
	}
}

void coder::Decoder::GetColorProc(size_t index, int __frame) {
	try {
		std::string coffecients;
		size_t      blocks;
		if (__frame == 0) {
			const size_t kBufferSize = ZSTD_getFrameContentSize(this->I_Frame_Patches_[index].colors_.c_str(), this->I_Frame_Patches_[index].colors_.size());

			if (kBufferSize == 0 || kBufferSize == ZSTD_CONTENTSIZE_UNKNOWN || kBufferSize == ZSTD_CONTENTSIZE_ERROR) {
				throw "Wrong buffer size.";
			}

			coffecients.resize(kBufferSize);

			// decompression
			const size_t kDecompressedSize =
			    ZSTD_decompress(const_cast<char*>(coffecients.c_str()), kBufferSize, this->I_Frame_Patches_[index].colors_.c_str(), this->I_Frame_Patches_[index].colors_.size());

			// if error?
			const size_t __error_code = ZSTD_isError(kDecompressedSize);
			if (__error_code != 0) {
				throw "Wrong decompressed string size.";
			}

			// free excess space
			coffecients.resize(kDecompressedSize);
			blocks = this->I_Frame_Patches_[index].block_number_;
		}
		else {
			const size_t kBufferSize = ZSTD_getFrameContentSize(this->P_Frame_Patches_[index].colors_.c_str(), this->P_Frame_Patches_[index].colors_.size());

			if (kBufferSize == 0 || kBufferSize == ZSTD_CONTENTSIZE_UNKNOWN || kBufferSize == ZSTD_CONTENTSIZE_ERROR) {
				throw "Wrong buffer size.";
			}

			coffecients.resize(kBufferSize);

			// decompression
			const size_t kDecompressedSize =
			    ZSTD_decompress(const_cast<char*>(coffecients.c_str()), kBufferSize, this->P_Frame_Patches_[index].colors_.c_str(), this->P_Frame_Patches_[index].colors_.size());

			// if error?
			const size_t __error_code = ZSTD_isError(kDecompressedSize);
			if (__error_code != 0) {
				throw "Wrong decompressed string size.";
			}

			// free excess space
			coffecients.resize(kDecompressedSize);
			blocks = this->P_Frame_Patches_[index].block_number_;
		}

		std::vector<std::vector<int>> coff_r(blocks, std::vector<int>(512, 0));
		std::vector<std::vector<int>> coff_g(blocks, std::vector<int>(512, 0));
		std::vector<std::vector<int>> coff_b(blocks, std::vector<int>(512, 0));

		std::vector<vvs::type::MacroBlock8> color_blocks(blocks);

		size_t coff_index = 0;
		for (size_t i = 0; i < blocks; i++) {
			coff_r[i][0] = vvs::operation::Char2Int(coffecients[coff_index], coffecients[coff_index + 1]);
			coff_index += 2;
		}
		for (size_t i = 0; i < blocks; i++) {
			for (size_t j = 1; j < 512; j++) {
				if (coffecients[coff_index] != static_cast<char>(-128)) {
					coff_r[i][j] = static_cast<int>(coffecients[coff_index]);
					coff_index++;
				}
				else {
					coff_index++;
					break;
				}
			}
		}

		for (size_t i = 0; i < blocks; i++) {
			coff_g[i][0] = vvs::operation::Char2Int(coffecients[coff_index], coffecients[coff_index + 1]);
			coff_index += 2;
		}
		for (size_t i = 0; i < blocks; i++) {
			for (size_t j = 1; j < 512; j++) {
				if (coffecients[coff_index] != static_cast<char>(-128)) {
					coff_g[i][j] = static_cast<int>(coffecients[coff_index]);
					coff_index++;
				}
				else {
					coff_index++;
					break;
				}
			}
		}

		for (size_t i = 0; i < blocks; i++) {
			coff_b[i][0] = vvs::operation::Char2Int(coffecients[coff_index], coffecients[coff_index + 1]);
			coff_index += 2;
		}
		for (size_t i = 0; i < blocks; i++) {
			for (size_t j = 1; j < 512; j++) {
				if (coffecients[coff_index] != static_cast<char>(-128)) {
					coff_b[i][j] = static_cast<int>(coffecients[coff_index]);
					coff_index++;
				}
				else {
					coff_index++;
					break;
				}
			}
		}

		for (size_t i = 0; i < blocks; i++) {
			color_blocks[i].RGBIDCT3(coff_r[i], coff_g[i], coff_b[i]);
		}

		if (__frame == 0) {
			for (size_t i = 0; i < blocks; i++) {
				for (size_t j = 0; j < 512; j++) {
					this->fitting_colors_[index].emplace_back(color_blocks[i].RGB_[j]);
				}
			}
		}
		else {
			for (size_t i = 0; i < blocks; i++) {
				for (size_t j = 0; j < 512; j++) {
					this->p_colors_[index].emplace_back(color_blocks[i].RGB_[j]);
				}
			}
			if (__frame == 1) {
				for (size_t i = 0; i < this->p_colors_[index].size(); i++) {
					this->p_colors_[index][i] += this->fitting_colors_[index][i];
				}
			}
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Zstd compression : " << error_message << std::endl;
		std::exit(1);
	}
}

#pragma GCC diagnostic pop