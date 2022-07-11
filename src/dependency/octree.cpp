/***
 * @Author: ChenRP07
 * @Date: 2022-06-30 14:33:25
 * @LastEditTime: 2022-07-11 11:09:22
 * @LastEditors: ChenRP07
 * @Description: Implement of octree
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
		this->tree_height_ = static_cast<size_t>(std::log2(this->tree_resolution_)) + 1;
		this->tree_nodes_.resize(this->tree_height_);

		// use a index vector to indicate if there is point in a octree node.
		std::vector<size_t> __octree_points(__point_cloud.size());
		for (size_t i = 0; i < __point_cloud.size(); i++) {
			__octree_points[i] = i;
		}

		this->colors_.resize(__point_colors.size());

		// add tree node recursively
		this->AddTreeNode(__point_cloud, __point_colors, __octree_points, 0, this->tree_resolution_, this->tree_center_);

		for (int i = this->tree_height_ - 2; i >= 0; i--) {
			size_t node_index = 0;
			for (size_t j = 0; j < this->tree_nodes_[i].size(); j++) {
				size_t count                    = vvs::operation::NodePointCount(this->tree_nodes_[i][j].subnodes_);
				this->tree_nodes_[i][j].weight_ = 0;
				for (size_t k = 0; k < count; k++) {
					this->tree_nodes_[i][j].weight_ += this->tree_nodes_[i + 1][node_index].weight_;
					node_index++;
				}
			}
		}
		// color compensation
		for (size_t i = 1; i < this->colors_.size(); i++) {
			for (size_t j = 0; j < this->colors_[i].size(); j++) {
				this->colors_[i][j] -= this->colors_[0][j];
			}
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
			return false;
		}
		else if (__height < this->tree_height_ - 1) {
			// new node
			vvs::type::TreeNode node;
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

			node.SetNode(vvs::operation::BoolSetToUChar(__node_value_bits));
			// add node to octree
			this->tree_nodes_[__height].emplace_back(node);
			return true;
		}
		// leave layer, add real color
		else if (__height == this->tree_height_ - 1) {
			vvs::type::TreeNode node(static_cast<size_t>(1));
			vvs::type::ColorRGB i_color;
			for (size_t i = 0; i < this->colors_.size(); i++) {
				// voxel downsampling
				vvs::type::ColorRGB color;
				for (auto& k : __node_points) {
					color += __point_colors[i][k];
				}
				color /= __node_points.size();

				// add color to colors_
				this->colors_[i].emplace_back(color);
				if (i == 0) {
					i_color = color;
				}
			}
			node.SetSignal(i_color);
			this->tree_nodes_[__height].emplace_back(node);
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
 * @description: using zstd to compress the tree nodes to __result, and get the tree parameters
 * @param {string&} __result
 * @param {PointXYZ&} __center
 * @param {size_t&} __tree_height
 * @return {*}
 */
void Octree3D::TreeCompression(std::string& __result, pcl::PointXYZ& __center, size_t& __tree_height) {
	try {
		// tree parameters
		__center      = this->tree_center_;
		__tree_height = this->tree_height_;

		// compress sorce string in __source
		std::string __source;

		// add tree node to __source
		for (size_t i = 0; i < this->tree_height_ - 1; i++) {
			for (auto& j : this->tree_nodes_[i]) {
				__source += static_cast<char>(j.subnodes_);
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
 * @description: RAHT in [1], output compressed bytestream to __result, quantization is kQStep
 * @param {string&} __result
 * @param {int} kQStep
 * @return {*}
 */
void Octree3D::RAHT(std::string& __result, size_t index, const int kQStep) {
	try {
		// for each layer
		for (int i = this->tree_height_ - 2; i >= 0; i--) {
			// index for next layer
			size_t node_index = 0;
			// for each node
			for (int j = 0; j < this->tree_nodes_[i].size(); j++) {
				// subnodes signals
				std::vector<float> gys(8, 0.0f), gus(8, 0.0f), gvs(8, 0.0f);
				// subnodes 1-bit positions
				std::vector<size_t> pos;
				// subnodes weights
				std::vector<size_t> weight(8, 0);

				// get 1-bit positions
				vvs::operation::NodePointPosition(this->tree_nodes_[i][j].subnodes_, pos);

				for (auto& k : pos) {
					// get 1-bit node weight
					weight[k] = this->tree_nodes_[i + 1][node_index].weight_;
					// get 1-bit node signals
					gys[k] = this->tree_nodes_[i + 1][node_index].sig_y_;
					gus[k] = this->tree_nodes_[i + 1][node_index].sig_u_;
					gvs[k] = this->tree_nodes_[i + 1][node_index].sig_v_;
					// index ++
					node_index++;
				}
				vvs::operation::NodeSignalMerge3D(gys, gus, gvs, weight, this->tree_nodes_[i][j]);
			}
		}

		// coffecients
		std::vector<float> coff_y, coff_u, coff_v;

		// final signals
		coff_y.emplace_back(this->tree_nodes_[0][0].sig_y_);
		coff_u.emplace_back(this->tree_nodes_[0][0].sig_u_);
		coff_v.emplace_back(this->tree_nodes_[0][0].sig_v_);

		// all coffecients
		for (size_t i = 0; i < this->tree_height_ - 1; i++) {
			for (size_t j = 0; j < this->tree_nodes_[i].size(); j++) {
				for (auto& k : this->tree_nodes_[i][j].cof_y_) {
					coff_y.emplace_back(k);
				}
				this->tree_nodes_[i][j].cof_y_.clear();
				for (auto& k : this->tree_nodes_[i][j].cof_u_) {
					coff_u.emplace_back(k);
				}
				this->tree_nodes_[i][j].cof_u_.clear();
				for (auto& k : this->tree_nodes_[i][j].cof_v_) {
					coff_v.emplace_back(k);
				}
				this->tree_nodes_[i][j].cof_v_.clear();
			}
		}

		// quantization
		std::vector<int> Qcoff_y, Qcoff_u, Qcoff_v;

		for (auto& k : coff_y) {
			Qcoff_y.emplace_back(static_cast<int>(std::round(k / kQStep)));
		}
		for (auto& k : coff_u) {
			Qcoff_u.emplace_back(static_cast<int>(std::round(k / kQStep)));
		}
		for (auto& k : coff_v) {
			Qcoff_v.emplace_back(static_cast<int>(std::round(k / kQStep)));
		}
		// std::ofstream outfile("./sig/Patch#" + std::to_string(index) + ".dat");
		// for (size_t i = 0; i < Qcoff_y.size(); i++) {
		// 	outfile << Qcoff_y[i] << " " << Qcoff_u[i] << " " << Qcoff_v[i] << std::endl;
		// }
		std::string temp;

		// first signal is 32-bit int
		temp += static_cast<char>(Qcoff_y[0] >> 24), temp += static_cast<char>(Qcoff_y[0] >> 16), temp += static_cast<char>(Qcoff_y[0] >> 8), temp += static_cast<char>(Qcoff_y[0]);
		// others are 16-bit int
		for (size_t i = 1; i < Qcoff_y.size(); i++) {
#ifdef _RAHT_FIX_16_
			temp += static_cast<char>(Qcoff_y[i] >> 8);
			temp += static_cast<char>(Qcoff_y[i]);
#endif
#ifdef _RAHT_FIX_8_
			temp += vvs::operation::Int2Char(Qcoff_y[i]);
#endif
		}

		temp += static_cast<char>(Qcoff_u[0] >> 24), temp += static_cast<char>(Qcoff_u[0] >> 16), temp += static_cast<char>(Qcoff_u[0] >> 8), temp += static_cast<char>(Qcoff_u[0]);
		for (size_t i = 1; i < Qcoff_u.size(); i++) {
#ifdef _RAHT_FIX_16_
			temp += static_cast<char>(Qcoff_u[i] >> 8);
			temp += static_cast<char>(Qcoff_u[i]);
#endif
#ifdef _RAHT_FIX_8_
			temp += vvs::operation::Int2Char(Qcoff_u[i]);
#endif
		}

		temp += static_cast<char>(Qcoff_v[0] >> 24), temp += static_cast<char>(Qcoff_v[0] >> 16), temp += static_cast<char>(Qcoff_v[0] >> 8), temp += static_cast<char>(Qcoff_v[0]);
		for (size_t i = 1; i < Qcoff_v.size(); i++) {
#ifdef _RAHT_FIX_16_
			temp += static_cast<char>(Qcoff_v[i] >> 8);
			temp += static_cast<char>(Qcoff_v[i]);
#endif
#ifdef _RAHT_FIX_8_
			temp += vvs::operation::Int2Char(Qcoff_v[i]);
#endif
		}

		// malloc some space
		const size_t kBufferSize = ZSTD_compressBound(temp.size());
		__result.resize(kBufferSize);

		// compression
		const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__result.c_str()), kBufferSize, temp.c_str(), temp.size(), ZSTD_maxCLevel());

		// if error?
		const size_t __error_code = ZSTD_isError(kCompressedSize);
		if (__error_code != 0) {
			throw "Wrong compressed string size.";
		}

		// delete excess space
		__result.resize(kCompressedSize);
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree3D RAHT : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: color compression, Y_DCs Y_AC EOB Y_AC EOB ... U_DCs U_AC EOB U_AC EOB ... V_DCs V_AC EOB V_AC EOB ...
 * @param {vector<string>&} __result;
 * @return {size_t} number of blocks
 */
void Octree3D::ColorCompression(std::vector<std::string>& __result, size_t index, int kQStep) {
	try {
		std::ofstream out1("./sig/Patch$" + std::to_string(index) + ".dat"), out2("./desig/Patch$" + std::to_string(index) + ".dat");

		for (size_t i = 0; i < this->colors_[0].size(); i++) {
			out1 << this->colors_[0][i].y_ << " " << this->colors_[0][i].u_ << " " << this->colors_[0][i].v_ << std::endl;
		}

		for (size_t i = 0; i < this->colors_.back().size(); i++) {
			out2 << this->colors_.back()[i].y_ << " " << this->colors_.back()[i].u_ << " " << this->colors_.back()[i].v_ << std::endl;
		}
		// allocate space
		__result.resize(this->colors_.size());

		for (size_t i = 0; i < colors_.size(); i++) {
			for (size_t j = 0; j < this->tree_nodes_.back().size(); j++) {
				this->tree_nodes_.back()[j].SetSignal(this->colors_[i][j]);
			}

			if (i == 0) {
				this->RAHT(__result[i], index, DEFAULT_KQSTEP);
			}
			else {
				this->RAHT(__result[i], index, DEFAULT_PKQSTEP);
			}
		}
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
		this->tree_height_ = static_cast<size_t>(std::log2(this->tree_resolution_)) + 1;
		this->tree_nodes_.resize(this->tree_height_);

		// use a index vector to indicate if there is point in a octree node.
		std::vector<size_t> __octree_points(__point_cloud.size());
		for (size_t i = 0; i < __point_cloud.size(); i++) {
			__octree_points[i] = i;
		}

		// add tree node recursively
		this->AddTreeNode(__point_cloud, __octree_points, 0, this->tree_resolution_, this->tree_center_);
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
			return false;
		}
		else if (__height < this->tree_height_ - 1) {
			// new node
			vvs::type::TreeNode node(__node_points.size());
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

			node.SetNode(vvs::operation::BoolSetToUChar(__node_value_bits));
			// add node to octree
			this->tree_nodes_[__height].emplace_back(node);
			return true;
		}
		// leave layer, add real color
		else if (__height == this->tree_height_ - 1) {
			vvs::type::TreeNode node(static_cast<size_t>(1));
			// voxel downsampling
			vvs::type::ColorRGB color;
			for (auto& k : __node_points) {
				vvs::type::ColorRGB temp(__point_cloud[k]);
				color += temp;
			}
			color /= __node_points.size();

			node.SetSignal(color);
			this->tree_nodes_[__height].emplace_back(node);

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
 * @description: using zstd to compress the tree nodes to __result, and get the tree parameters
 * @param {string&} __result
 * @param {PointXYZ&} __center
 * @param {size_t&} __tree_height
 * @return {*}
 */
void SingleOctree3D::TreeCompression(std::string& __result, pcl::PointXYZ& __center, size_t& __tree_height) {
	try {
		// tree parameters
		__center      = this->tree_center_;
		__tree_height = this->tree_height_;

		// compress sorce string in __source
		std::string __source;

		// add tree node to __source
		for (size_t i = 0; i < this->tree_height_ - 1; i++) {
			for (auto& j : this->tree_nodes_[i]) {
				__source += static_cast<char>(j.subnodes_);
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
 * @description: RAHT in [1], output compressed bytestream to __result, quantization is kQStep
 * @param {string&} __result
 * @param {int} kQStep
 * @return {*}
 */
void SingleOctree3D::RAHT(std::string& __result, const int kQStep) {
	try {
		// for each layer
		for (int i = this->tree_height_ - 2; i >= 0; i--) {
			// index for next layer
			size_t node_index = 0;
			// for each node
			for (int j = 0; j < this->tree_nodes_[i].size(); j++) {
				// subnodes signals
				std::vector<float> gys(8, 0.0f), gus(8, 0.0f), gvs(8, 0.0f);
				// subnodes 1-bit positions
				std::vector<size_t> pos;
				// subnodes weights
				std::vector<size_t> weight(8, 0);

				// get 1-bit positions
				vvs::operation::NodePointPosition(this->tree_nodes_[i][j].subnodes_, pos);

				for (auto& k : pos) {
					// get 1-bit node weight
					weight[k] = this->tree_nodes_[i + 1][node_index].weight_;
					// get 1-bit node signals
					gys[k] = this->tree_nodes_[i + 1][node_index].sig_y_;
					gus[k] = this->tree_nodes_[i + 1][node_index].sig_u_;
					gvs[k] = this->tree_nodes_[i + 1][node_index].sig_v_;
					// index ++
					node_index++;
				}
				vvs::operation::NodeSignalMerge3D(gys, gus, gvs, weight, this->tree_nodes_[i][j]);
			}
		}

		// coffecients
		std::vector<float> coff_y, coff_u, coff_v;

		// final signals
		coff_y.emplace_back(this->tree_nodes_[0][0].sig_y_);
		coff_u.emplace_back(this->tree_nodes_[0][0].sig_u_);
		coff_v.emplace_back(this->tree_nodes_[0][0].sig_v_);

		// all coffecients
		for (size_t i = 0; i < this->tree_height_ - 1; i++) {
			for (size_t j = 0; j < this->tree_nodes_[i].size(); j++) {
				for (auto& k : this->tree_nodes_[i][j].cof_y_) {
					coff_y.emplace_back(k);
				}
				for (auto& k : this->tree_nodes_[i][j].cof_u_) {
					coff_u.emplace_back(k);
				}
				for (auto& k : this->tree_nodes_[i][j].cof_v_) {
					coff_v.emplace_back(k);
				}
			}
		}

		// quantization
		std::vector<int> Qcoff_y, Qcoff_u, Qcoff_v;

		for (auto& k : coff_y) {
			Qcoff_y.emplace_back(static_cast<int>(std::round(k / kQStep)));
		}
		for (auto& k : coff_u) {
			Qcoff_u.emplace_back(static_cast<int>(std::round(k / kQStep)));
		}
		for (auto& k : coff_v) {
			Qcoff_v.emplace_back(static_cast<int>(std::round(k / kQStep)));
		}

		std::string temp;

		// first signal is 32-bit int
		temp += static_cast<char>(Qcoff_y[0] >> 24), temp += static_cast<char>(Qcoff_y[0] >> 16), temp += static_cast<char>(Qcoff_y[0] >> 8), temp += static_cast<char>(Qcoff_y[0]);
		// others are 16-bit int
		for (size_t i = 1; i < Qcoff_y.size(); i++) {
			temp += static_cast<char>(Qcoff_y[i] >> 8), temp += static_cast<char>(Qcoff_y[i]);
		}

		temp += static_cast<char>(Qcoff_u[0] >> 24), temp += static_cast<char>(Qcoff_u[0] >> 16), temp += static_cast<char>(Qcoff_u[0] >> 8), temp += static_cast<char>(Qcoff_u[0]);
		for (size_t i = 1; i < Qcoff_u.size(); i++) {
			temp += static_cast<char>(Qcoff_u[i] >> 8), temp += static_cast<char>(Qcoff_u[i]);
		}

		temp += static_cast<char>(Qcoff_v[0] >> 24), temp += static_cast<char>(Qcoff_v[0] >> 16), temp += static_cast<char>(Qcoff_v[0] >> 8), temp += static_cast<char>(Qcoff_v[0]);
		for (size_t i = 1; i < Qcoff_v.size(); i++) {
			temp += static_cast<char>(Qcoff_v[i] >> 8), temp += static_cast<char>(Qcoff_v[i]);
		}

		// malloc some space
		const size_t kBufferSize = ZSTD_compressBound(temp.size());
		__result.resize(kBufferSize);

		// compression
		const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__result.c_str()), kBufferSize, temp.c_str(), temp.size(), ZSTD_maxCLevel());

		// if error?
		const size_t __error_code = ZSTD_isError(kCompressedSize);
		if (__error_code != 0) {
			throw "Wrong compressed string size.";
		}

		// delete excess space
		__result.resize(kCompressedSize);
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in Octree3D RAHT : " << error_message << std::endl;
		std::exit(1);
	}
}