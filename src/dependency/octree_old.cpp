/***
 * @Author: ChenRP07
 * @Date: 2022-06-28 09:57:07
 * @LastEditTime: 2022-06-30 14:48:03
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "dependency/octree.h"
using namespace vvs::octree;

/***
 * @description: constructor, __res is the min resolution, default is 2.0
 * @param {float} __res
 * @return {*}
 */
Octree3D::Octree3D(const float __res) : kMinResolution{__res} {}

/***
 * @description: use __point_cloud and __point_colors to create a octree, cloud center is __center and span range is __res
 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
 * @param {const vector<vector<ColorRGB>>&} __point_colors
 * @param {PointXYZ} __center
 * @param {float} __res
 * @return {*}
 */
void Octree3D::SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const std::vector<std::vector<vvs::type::ColorRGB>>& __point_colors) {
	try {
		// cloud must be not empty
		if (__point_cloud.empty()) {
			throw "Empty input point cloud.";
		}

		// set center and resolution
		float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
		float max_x = FLT_MIN, max_y = FLT_MIN, max_z = FLT_MIN;
		for (auto& i : __point_cloud) {
			min_x = i.x < min_x ? i.x : min_x;
			min_y = i.y < min_y ? i.y : min_y;
			min_z = i.z < min_z ? i.z : min_z;
			max_x = i.x > max_x ? i.x : max_x;
			max_y = i.y > max_y ? i.y : max_y;
			max_z = i.z > max_z ? i.z : max_z;
		}

		float __resolution     = std::max(std::max(max_x - min_x, max_y - min_y), max_z - min_z);
		this->tree_resolution_ = std::pow(2.0f, std::ceil(std::log2(__resolution)));
		this->tree_center_.x   = (max_x + min_x) / 2.0f;
		this->tree_center_.y   = (max_y + min_y) / 2.0f;
		this->tree_center_.z   = (max_z + min_z) / 2.0f;

		// tree height is log2(resolution)
		this->tree_height_ = static_cast<size_t>(std::log2(this->tree_resolution_));
		this->tree_nodes_.resize(this->tree_height_);

		// use a index vector to indicate if there is point in a octree node.
		std::vector<size_t> __octree_points(__point_cloud.size());
		for (size_t i = 0; i < __point_cloud.size(); i++) {
			__octree_points[i] = i;
		}

		// allocate space for colors
		this->tree_colors_.resize(__point_colors.size(), std::vector<vvs::type::MacroBlock8>(1, vvs::type::MacroBlock8()));

		// add tree node recursively
		this->AddTreeNode(__point_cloud, __point_colors, __octree_points, 0, this->tree_resolution_, this->tree_center_);

		for (auto& i : this->tree_colors_) {
			i.back().Fill();
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree3D SetPointCloud : " << error_message << std::endl;
		std::exit(1);
	}
}

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
bool Octree3D::AddTreeNode(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const std::vector<std::vector<vvs::type::ColorRGB>>& __point_colors, std::vector<size_t>& __node_points,
                           size_t __height, float __res, pcl::PointXYZ __center) {
	try {
		// if this node is empty, return 0
		if (__node_points.empty()) {
			// res 1 is empty, add 1 empty color
			if (__height == this->tree_height_) {
				// for each frame, first check the last block is full ? if full, add a new block
				for (auto& i : this->tree_colors_) {
					if (i.back().full()) {
						i.emplace_back();
					}
					i.back().PushBack(1);
				}
			}
			// res 2 is empty, add 8 empty color
			else if (__height == (this->tree_height_ - 1)) {
				for (auto& i : this->tree_colors_) {
					if (i.back().full()) {
						i.emplace_back();
					}
					i.back().PushBack(8);
				}
			}
			// res 4 is empty, add 64 empty color
			else if (__height == (this->tree_height_ - 2)) {
				for (auto& i : this->tree_colors_) {
					if (i.back().full()) {
						i.emplace_back();
					}
					i.back().PushBack(64);
				}
			}
			return false;
		}
		// leave layer, add real color
		else if (__height == this->tree_height_) {
			for (size_t i = 0; i < this->tree_colors_.size(); i++) {
				// voxel downsampling
				vvs::type::ColorRGB color;
				for (auto& k : __node_points) {
					color += __point_colors[i][k];
				}
				color /= __node_points.size();
				// check if the last block is empty
				if (this->tree_colors_[i].back().full()) {
					this->tree_colors_[i].emplace_back();
				}
				this->tree_colors_[i].back().PushBack(color);
			}
			return true;
		}
		else if (__height < this->tree_height_) {
			// for 8 subnodes
			std::vector<std::vector<size_t>> __subnodes_value(8, std::vector<size_t>());

			// calculate the position
			for (auto& i : __node_points) {
				int pos = 0;
				pos |= __point_cloud[i].x > __center.x ? 0 : 1;
				pos <<= 1;
				pos |= __point_cloud[i].y > __center.y ? 0 : 1;
				pos <<= 1;
				pos |= __point_cloud[i].z > __center.z ? 0 : 1;
				__subnodes_value[pos].emplace_back(i);
			}

			__node_points.clear();

			// calculte the node value
			std::vector<bool> __node_value_bits(8, false);
			for (size_t i = 0; i < 8; i++) {
				// get a new subcenter
				pcl::PointXYZ __new_center = vvs::operation::SubnodeCenter(__center, i, __res / 2);
				__node_value_bits[i]       = this->AddTreeNode(__point_cloud, __point_colors, __subnodes_value[i], __height + 1, __res / 2, __new_center);
			}

			// add node to octree
			this->tree_nodes_[__height].push_back(vvs::operation::BoolSetToUChar(__node_value_bits));
			return true;
		}
		else {
			throw "Height out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree3D AddTreeNode : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: using zstd to compress the tree nodes to __result
 * @param {string&} __result
 * @return {*}
 */
void Octree3D::TreeCompression(std::string& __result) {
	try {
		// compress sorce string in __source
		std::string __source;

		// add tree node to __source
		for (auto& i : this->tree_nodes_) {
			for (auto& j : i) {
				__source += static_cast<char>(j);
			}
		}

		// __source cannot be empty
		if (__source.empty()) {
			throw "Empty source string.";
		}

		// malloc some space
		const size_t kBufferSize = ZSTD_compressBound(__source.size());
		__result.resize(kBufferSize);

		// compression
		const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__result.c_str()), kBufferSize, __source.c_str(), __source.size(), ZSTD_maxCLevel());

		// if error?
		const size_t __error_code = ZSTD_isError(kCompressedSize);
		if (__error_code != 0) {
			throw "Wrong compressed string size.";
		}

		// delete excess space
		__result.resize(kCompressedSize);
	}
	catch (const char* error_message) {
		std::cerr << "Fatal erro in Octree3D TreeCompression : " << std::endl;
		std::exit(1);
	}
}

/***
 * @description: color compression, Y_DCs Y_AC EOB Y_AC EOB ... U_DCs U_AC EOB U_AC EOB ... V_DCs V_AC EOB V_AC EOB ...
 * @param {vector<string>&} __result;
 * @return {size_t} number of blocks
 */
size_t Octree3D::ColorCompression(std::vector<std::string>& __result) {
	try {
		// allocate space
		__result.resize(this->tree_colors_.size());
		// convert rgb to yuv and down sampling
		for (auto& i : this->tree_colors_) {
			for (auto& j : i) {
				j.RGB2YUV820();
			}
		}

		// color compensation
		for (size_t i = 1; i < this->tree_colors_.size(); i++) {
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				this->tree_colors_[i][j] -= this->tree_colors_[0][j];
			}
		}

		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			std::vector<std::vector<int>> __coefficients_y(this->tree_colors_[i].size()), __coefficients_u(this->tree_colors_[i].size()), __coefficients_v(this->tree_colors_[i].size());
			// DCT quantization and zigzag-scan for each block.
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				this->tree_colors_[i][j].YUVDCT3(__coefficients_y[j], __coefficients_u[j], __coefficients_v[j]);
			}

			std::string source;

			// scan the y, first DC is represented by 16-bit int
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				source += vvs::operation::Int2CharH(__coefficients_y[j][0]);
				source += vvs::operation::Int2CharL(__coefficients_y[j][0]);
			}

			// ACs are represented by 8-bit int
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				// behind count, all ACs are 0;
				size_t count = 0;
				for (size_t k = 511; k >= 1; k--) {
					if (__coefficients_y[j][k] != 0) {
						count = k;
						break;
					}
				}
				for (size_t k = 1; k <= count; k++) {
					source += vvs::operation::Int2Char(__coefficients_y[j][k]);
				}

				source += static_cast<char>(-128);
			}

			// scan the u
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				source += vvs::operation::Int2CharH(__coefficients_u[j][0]);
				source += vvs::operation::Int2CharL(__coefficients_u[j][0]);
			}

			// ACs are represented by 8-bit int
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				// behind count, all ACs are 0;
				size_t count = 0;
				for (size_t k = 63; k >= 1; k--) {
					if (__coefficients_y[j][k] != 0) {
						count = k;
						break;
					}
				}
				for (size_t k = 1; k <= count; k++) {
					source += vvs::operation::Int2Char(__coefficients_y[j][k]);
				}

				source += static_cast<char>(-128);
			}

			// scan the v
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				source += vvs::operation::Int2CharH(__coefficients_v[j][0]);
				source += vvs::operation::Int2CharL(__coefficients_v[j][0]);
			}

			// ACs are represented by 8-bit int
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				// behind count, all ACs are 0;
				size_t count = 0;
				for (size_t k = 63; k >= 1; k--) {
					if (__coefficients_y[j][k] != 0) {
						count = k;
						break;
					}
				}
				for (size_t k = 1; k <= count; k++) {
					source += vvs::operation::Int2Char(__coefficients_y[j][k]);
				}

				source += static_cast<char>(-128);
			}

			// compress
			// malloc some space
			const size_t kBufferSize = ZSTD_compressBound(source.size());
			__result[i].resize(kBufferSize);

			// compression
			const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__result[i].c_str()), kBufferSize, source.c_str(), source.size(), ZSTD_maxCLevel());

			// if error?
			const size_t __error_code = ZSTD_isError(kCompressedSize);
			if (__error_code != 0) {
				throw "Wrong compressed string size.";
			}
			__result[i].resize(kCompressedSize);
		}

		return this->tree_colors_[0].size();
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree ColorCompression : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: constructor, __res is the min resolution, default is 2.0
 * @param {float} __res
 * @return {*}
 */
SingleOctree3D::SingleOctree3D(const float __res) : kMinResolution{__res} {}

/***
 * @description: use __point_cloud and __point_colors to create a octree, cloud center is __center and span range is __res
 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
 * @param {PointXYZ} __center
 * @param {float} __res
 * @return {*}
 */
void SingleOctree3D::SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud) {
	try {
		// cloud must be not empty
		if (__point_cloud.empty()) {
			throw "Empty input point cloud.";
		}

		// set center and resolution
		float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
		float max_x = FLT_MIN, max_y = FLT_MIN, max_z = FLT_MIN;
		for (auto& i : __point_cloud) {
			min_x = i.x < min_x ? i.x : min_x;
			min_y = i.y < min_y ? i.y : min_y;
			min_z = i.z < min_z ? i.z : min_z;
			max_x = i.x > max_x ? i.x : max_x;
			max_y = i.y > max_y ? i.y : max_y;
			max_z = i.z > max_z ? i.z : max_z;
		}

		float __resolution     = std::max(std::max(max_x - min_x, max_y - min_y), max_z - min_z);
		this->tree_resolution_ = std::pow(2.0f, std::ceil(std::log2(__resolution)));
		this->tree_center_.x   = (max_x + min_x) / 2.0f;
		this->tree_center_.y   = (max_y + min_y) / 2.0f;
		this->tree_center_.z   = (max_z + min_z) / 2.0f;

		// tree height is log2(resolution)
		this->tree_height_ = static_cast<size_t>(std::log2(this->tree_resolution_));
		this->tree_nodes_.resize(this->tree_height_);

		// use a index vector to indicate if there is point in a octree node.
		std::vector<size_t> __octree_points(__point_cloud.size());
		for (size_t i = 0; i < __point_cloud.size(); i++) {
			__octree_points[i] = i;
		}

		// allocate space for colors
		this->tree_colors_.resize(1, vvs::type::MacroBlock8());

		// add tree node recursively
		this->AddTreeNode(__point_cloud, __octree_points, 0, this->tree_resolution_, this->tree_center_);
		this->tree_colors_.back().Fill();
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree3D SetPointCloud : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: add a tree node on __height layer and this node res is __res, center is __center, point indexes are __node_points
 * @param {const PointCloud<PointXYZRGB>&} __point_cloud
 * @param {vector<size_t>&} __node_points
 * @param {size_t} __height
 * @param {PointXYZ} __center
 * @param {float} __res
 * @return {*}
 */
bool SingleOctree3D::AddTreeNode(const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, std::vector<size_t>& __node_points, size_t __height, float __res, pcl::PointXYZ __center) {
	try {
		// if this node is empty, return 0
		if (__node_points.empty()) {
			// res 1 is empty, add 1 empty color
			if (__height == this->tree_height_) {
				// first check the last block is full ? if full, add a new block
				if (this->tree_colors_.back().full()) {
					this->tree_colors_.emplace_back();
				}
				this->tree_colors_.back().PushBack(1);
			}
			// res 2 is empty, add 8 empty color
			else if (__height == (this->tree_height_ - 1)) {
				if (__height == this->tree_height_) {
					// first check the last block is full ? if full, add a new block
					if (this->tree_colors_.back().full()) {
						this->tree_colors_.emplace_back();
					}
					this->tree_colors_.back().PushBack(8);
				}
			}
			// res 4 is empty, add 64 empty color
			else if (__height == (this->tree_height_ - 2)) {
				if (__height == this->tree_height_) {
					// first check the last block is full ? if full, add a new block
					if (this->tree_colors_.back().full()) {
						this->tree_colors_.emplace_back();
					}
					this->tree_colors_.back().PushBack(64);
				}
			}
			return false;
		}
		// leave layer, add real color
		else if (__height == this->tree_height_) {
			// voxel downsampling
			vvs::type::ColorRGB color;
			for (auto& k : __node_points) {
				vvs::type::ColorRGB temp(__point_cloud[k]);
				color += temp;
			}
			color /= __node_points.size();
			// check if the last block is empty
			if (this->tree_colors_.back().full()) {
				this->tree_colors_.emplace_back();
			}
			this->tree_colors_.back().PushBack(color);

			return true;
		}
		else if (__height < this->tree_height_) {
			// for 8 subnodes
			std::vector<std::vector<size_t>> __subnodes_value(8, std::vector<size_t>());

			// calculate the position
			for (auto& i : __node_points) {
				int pos = 0;
				pos |= __point_cloud[i].x > __center.x ? 0 : 1;
				pos <<= 1;
				pos |= __point_cloud[i].y > __center.y ? 0 : 1;
				pos <<= 1;
				pos |= __point_cloud[i].z > __center.z ? 0 : 1;
				__subnodes_value[pos].emplace_back(i);
			}

			__node_points.clear();

			// calculte the node value
			std::vector<bool> __node_value_bits(8, false);
			for (size_t i = 0; i < 8; i++) {
				// get a new subcenter
				pcl::PointXYZ __new_center = vvs::operation::SubnodeCenter(__center, i, __res / 2);
				__node_value_bits[i]       = this->AddTreeNode(__point_cloud, __subnodes_value[i], __height + 1, __res / 2, __new_center);
			}

			// add node to octree
			this->tree_nodes_[__height].push_back(vvs::operation::BoolSetToUChar(__node_value_bits));
			return true;
		}
		else {
			throw "Height out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree3D AddTreeNode : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: using zstd to compress the tree nodes to __result
 * @param {string&} __result
 * @return {*}
 */
void SingleOctree3D::TreeCompression(std::string& __result) {
	try {
		// compress sorce string in __source
		std::string __source;

		// add tree node to __source
		for (auto& i : this->tree_nodes_) {
			for (auto& j : i) {
				__source += static_cast<char>(j);
			}
		}

		// __source cannot be empty
		if (__source.empty()) {
			throw "Empty source string.";
		}

		// malloc some space
		const size_t kBufferSize = ZSTD_compressBound(__source.size());
		__result.resize(kBufferSize);

		// compression
		const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__result.c_str()), kBufferSize, __source.c_str(), __source.size(), ZSTD_maxCLevel());

		// if error?
		const size_t __error_code = ZSTD_isError(kCompressedSize);
		if (__error_code != 0) {
			throw "Wrong compressed string size.";
		}

		// delete excess space
		__result.resize(kCompressedSize);
	}
	catch (const char* error_message) {
		std::cerr << "Fatal erro in Octree3D TreeCompression : " << std::endl;
		std::exit(1);
	}
}

/***
 * @description: color compression, Y_DCs Y_AC EOB Y_AC EOB ... U_DCs U_AC EOB U_AC EOB ... V_DCs V_AC EOB V_AC EOB ...
 * @param {string&} __result;
 * @return {size_t} number of blocks
 */
size_t SingleOctree3D::ColorCompression(std::string& __result) {
	try {
		// convert rgb to yuv and down sampling
		for (auto& i : this->tree_colors_) {
			i.RGB2YUV820();
		}

		std::vector<std::vector<int>> __coefficients_y(this->tree_colors_.size()), __coefficients_u(this->tree_colors_.size()), __coefficients_v(this->tree_colors_.size());
		// DCT quantization and zigzag-scan for each block.
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			this->tree_colors_[i].YUVDCT3(__coefficients_y[i], __coefficients_u[i], __coefficients_v[i]);
		}

		std::string source;

		// scan the y, first DC is represented by 16-bit int
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			source += vvs::operation::Int2CharH(__coefficients_y[i][0]);
			source += vvs::operation::Int2CharL(__coefficients_y[i][0]);
		}

		// ACs are represented by 8-bit int
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			// behind count, all ACs are 0;
			size_t count = 0;
			for (size_t k = 511; k >= 1; k--) {
				if (__coefficients_y[i][k] != 0) {
					count = k;
					break;
				}
			}
			for (size_t k = 1; k <= count; k++) {
				source += vvs::operation::Int2Char(__coefficients_y[i][k]);
			}

			source += static_cast<char>(-128);
		}

		// scan the u
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			source += vvs::operation::Int2CharH(__coefficients_u[i][0]);
			source += vvs::operation::Int2CharL(__coefficients_u[i][0]);
		}

		// ACs are represented by 8-bit int
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			// behind count, all ACs are 0;
			size_t count = 0;
			for (size_t k = 63; k >= 1; k--) {
				if (__coefficients_y[i][k] != 0) {
					count = k;
					break;
				}
			}
			for (size_t k = 1; k <= count; k++) {
				source += vvs::operation::Int2Char(__coefficients_y[i][k]);
			}

			source += static_cast<char>(-128);
		}

		// scan the v
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			source += vvs::operation::Int2CharH(__coefficients_v[i][0]);
			source += vvs::operation::Int2CharL(__coefficients_v[i][0]);
		}

		// ACs are represented by 8-bit int
		for (size_t i = 0; i < this->tree_colors_.size(); i++) {
			// behind count, all ACs are 0;
			size_t count = 0;
			for (size_t k = 63; k >= 1; k--) {
				if (__coefficients_y[i][k] != 0) {
					count = k;
					break;
				}
			}
			for (size_t k = 1; k <= count; k++) {
				source += vvs::operation::Int2Char(__coefficients_y[i][k]);
			}

			source += static_cast<char>(-128);
		}

		// compress
		// malloc some space
		const size_t kBufferSize = ZSTD_compressBound(source.size());
		__result.resize(kBufferSize);

		// compression
		const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__result.c_str()), kBufferSize, source.c_str(), source.size(), ZSTD_maxCLevel());

		// if error?
		const size_t __error_code = ZSTD_isError(kCompressedSize);
		if (__error_code != 0) {
			throw "Wrong compressed string size.";
		}
		__result.resize(kCompressedSize);

		return this->tree_colors_.size();
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree ColorCompression : " << error_message << std::endl;
		std::exit(1);
	}
}