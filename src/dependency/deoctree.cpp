/***
 * @Author: ChenRP07
 * @Date: 2022-07-04 11:35:23
 * @LastEditTime: 2022-07-12 10:23:29
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "dependency/octree.h"
using namespace vvs::octree;

/***
 * @description: constructor
 * @param {float} __min_res
 * @return {*}
 */
DeOctree3D::DeOctree3D(const float __min_res) : kMinResolution{__min_res} {}

/***
 * @description: set tree center
 * @param {const PointXYZ&} __center
 * @return {*}
 */
void DeOctree3D::SetCenter(const pcl::PointXYZ& __center) {
	this->tree_center_ = __center;
}

/***
 * @description: set tree height
 * @param {const size_t} __height
 * @return {*}
 */
void DeOctree3D::SetHeight(const size_t __height) {
	this->tree_height_ = __height - 1;
}

/***
 * @description: decompress __tree using zstd, set tree nodes and calculate node centers
 * @param {string&} __tree
 * @return {*}
 */
void DeOctree3D::SetTree(std::string& __tree) {
	try {
		// zstd decompress
		std::string nodes;

		// max size
		const size_t kBufferSize = ZSTD_getFrameContentSize(__tree.c_str(), __tree.size());
		if (kBufferSize == 0 || kBufferSize == ZSTD_CONTENTSIZE_UNKNOWN || kBufferSize == ZSTD_CONTENTSIZE_ERROR) {
			throw "Wrong buffer size.";
		}

		// reserve space
		nodes.resize(kBufferSize);

		// decompress
		const size_t kDecompressedSize = ZSTD_decompress(const_cast<char*>(nodes.c_str()), kBufferSize, __tree.c_str(), __tree.size());

		// if error?
		const size_t __error_code = ZSTD_isError(kDecompressedSize);
		if (__error_code != 0) {
			throw "Wrong decompressed string size.";
		}

		// free excess space
		nodes.resize(kDecompressedSize);

		// allocate space
		this->tree_nodes_.resize(this->tree_height_);

		// how many nodes in this layer
		size_t __layer_node_count = 1;
		// index for element in nodes
		size_t __nodes_string_index = 0;

		// for each layer
		for (size_t i = 0; i < this->tree_height_; i++) {
			// how many nodes in next layer
			size_t new_node_cnt = 0;
			// for each node
			for (size_t j = 0; j < __layer_node_count; j++) {
				if (__nodes_string_index >= nodes.size()) {
					throw "Out of nodes string range.";
				}
				// convert to uint8_t
				uint8_t data = static_cast<uint8_t>(nodes[__nodes_string_index]);

				// how many 1-bits in this node
				new_node_cnt += vvs::operation::NodePointCount(data);

				// add this node to tree
				this->tree_nodes_[i].emplace_back(data);

				if (i == this->tree_height_ - 1) {
					this->tree_nodes_[i].back().SetWeight(vvs::operation::NodePointCount(data));
				}
				// element index + 1
				__nodes_string_index++;
			}
			// new layer node count
			__layer_node_count = new_node_cnt;
		}

		// nodes should be scanned to the end
		if (__nodes_string_index != nodes.size()) {
			throw "String is too long.";
		}

		// calculate point for each layer
		this->tree_points_.resize(this->tree_height_, std::vector<pcl::PointXYZ>());

		// tree center is the only point in first layer
		this->tree_points_[0].emplace_back(this->tree_center_);

		// tree resolution is MinRes^tree_height_
		this->tree_resolution_ = std::pow(this->kMinResolution, this->tree_height_);

		float Res = this->tree_resolution_;

		// for each layer except the first one
		for (size_t i = 0; i < this->tree_height_ - 1; i++) {
			// for each node
			for (size_t j = 0; j < this->tree_nodes_[i].size(); j++) {
				// 1-bits position
				std::vector<size_t> pos;
				vvs::operation::NodePointPosition(this->tree_nodes_[i][j].subnodes_, pos);

				// for each 1-bit
				for (auto& k : pos) {
					// calculate subnode center, add it to next layer
					pcl::PointXYZ subnode_center = vvs::operation::SubnodeCenter(this->tree_points_[i][j], k, Res / 2);
					this->tree_points_[i + 1].emplace_back(subnode_center);
				}
			}
			// resolution of next layer is half of this layer
			Res /= 2;
		}

		// updata all node weights
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
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in DeOctree3D SetTree : " << error_message << std::endl;
		std::exit(1);
	}
}

void DeOctree3D::IRAHT(std::string& __source, size_t point_count, std::vector<vvs::type::ColorYUV>& __result, size_t index, const int kQStep) {
	std::string  temp;
	const size_t kBufferSize = ZSTD_getFrameContentSize(__source.c_str(), __source.size());
	if (kBufferSize == 0 || kBufferSize == ZSTD_CONTENTSIZE_UNKNOWN || kBufferSize == ZSTD_CONTENTSIZE_ERROR) {
		throw "Wrong buffer size.";
	}

	// reserve space
	temp.resize(kBufferSize);

	// decompress
	const size_t kDecompressedSize = ZSTD_decompress(const_cast<char*>(temp.c_str()), kBufferSize, __source.c_str(), __source.size());

	// if error?
	const size_t __error_code = ZSTD_isError(kDecompressedSize);
	if (__error_code != 0) {
		throw "Wrong decompressed string size.";
	}

	// free excess space
	temp.resize(kDecompressedSize);

	// clear
	for (size_t i = 0; i < this->tree_nodes_.size(); i++) {
		for (size_t j = 0; j < this->tree_nodes_[i].size(); j++) {
			this->tree_nodes_[i][j].cof_y_.clear();
			this->tree_nodes_[i][j].cof_u_.clear();
			this->tree_nodes_[i][j].cof_v_.clear();
		}
	}
	std::vector<float> coffs_y, coffs_u, coffs_v;

	// scan coffs
	size_t cofs_size  = point_count;
	size_t cofs_index = 0;
	coffs_y.emplace_back(vvs::operation::Char2Float4(temp[cofs_index], temp[cofs_index + 1], temp[cofs_index + 2], temp[cofs_index + 3]));
	cofs_index += 4;

	for (size_t j = 1; j < cofs_size; j++) {
#ifdef _RAHT_FIX_16_
		coffs_y.emplace_back(static_cast<float>(vvs::operation::Char2Int(temp[cofs_index], temp[cofs_index + 1])));
		cofs_index += 2;
#endif
#ifdef _RAHT_FIX_8_
		if (temp[cofs_index] == static_cast<char>(0x80)) {
			int count = vvs::operation::Char2Int(temp[cofs_index + 1], temp[cofs_index + 2]);
			for (int k = 0; k < count; k++) {
				coffs_y.emplace_back(0.0f);
			}
			j += (count - 1);
			cofs_index += 3;
		}
		else {
			coffs_y.emplace_back(static_cast<float>(temp[cofs_index]));
			cofs_index++;
		}
#endif
	}

	coffs_u.emplace_back(vvs::operation::Char2Float4(temp[cofs_index], temp[cofs_index + 1], temp[cofs_index + 2], temp[cofs_index + 3]));
	cofs_index += 4;

	for (size_t j = 1; j < cofs_size; j++) {
#ifdef _RAHT_FIX_16_
		coffs_u.emplace_back(static_cast<float>(vvs::operation::Char2Int(temp[cofs_index], temp[cofs_index + 1])));
		cofs_index += 2;
#endif
#ifdef _RAHT_FIX_8_
		if (temp[cofs_index] == static_cast<char>(0x80)) {
			int count = vvs::operation::Char2Int(temp[cofs_index + 1], temp[cofs_index + 2]);
			for (int k = 0; k < count; k++) {
				coffs_u.emplace_back(0.0f);
			}
			j += (count - 1);
			cofs_index += 3;
		}
		else {
			coffs_u.emplace_back(static_cast<float>(temp[cofs_index]));
			cofs_index++;
		}
#endif
	}

	coffs_v.emplace_back(vvs::operation::Char2Float4(temp[cofs_index], temp[cofs_index + 1], temp[cofs_index + 2], temp[cofs_index + 3]));
	cofs_index += 4;

	for (size_t j = 1; j < cofs_size; j++) {
#ifdef _RAHT_FIX_16_
		coffs_v.emplace_back(static_cast<float>(vvs::operation::Char2Int(temp[cofs_index], temp[cofs_index + 1])));
		cofs_index += 2;
#endif
#ifdef _RAHT_FIX_8_
		if (temp[cofs_index] == static_cast<char>(0x80)) {
			int count = vvs::operation::Char2Int(temp[cofs_index + 1], temp[cofs_index + 2]);
			for (int k = 0; k < count; k++) {
				coffs_v.emplace_back(0.0f);
			}
			j += (count - 1);
			cofs_index += 3;
		}
		else {
			coffs_v.emplace_back(static_cast<float>(temp[cofs_index]));
			cofs_index++;
		}
#endif
	}
	// std::ofstream outfile("./desig/Patch$" + std::to_string(index) + ".dat");
	// for (size_t i = 0; i < coffs_y.size(); i++) {
	// 	outfile << coffs_y[i] << " " << coffs_u[i] << " " << coffs_v[i] << std::endl;
	// }
	// dequantization
	for (auto& i : coffs_y) {
		i *= kQStep;
	}
	for (auto& i : coffs_u) {
		i *= kQStep;
	}
	for (auto& i : coffs_v) {
		i *= kQStep;
	}

	// final signal
	this->tree_nodes_[0][0].sig_y_ = coffs_y[0];
	this->tree_nodes_[0][0].sig_u_ = coffs_u[0];
	this->tree_nodes_[0][0].sig_v_ = coffs_v[0];

	// index for coff
	size_t coffs_scan_index = 1;

	// point count
	size_t point_cnt = 0;
	// fore each layer
	for (size_t i = 0; i < this->tree_height_; i++) {
		// for each node
		// subnode index in next layer
		size_t next_layer_index = 0;
		for (size_t j = 0; j < this->tree_nodes_[i].size(); j++) {
			// 1-bits position
			std::vector<size_t> pos;
			vvs::operation::NodePointPosition(this->tree_nodes_[i][j].subnodes_, pos);

			// non-empty subnodes weight
			std::vector<size_t> weight(8, 0);
			if (i != tree_height_ - 1) {
				for (size_t k = 0; k < pos.size(); k++) {
					weight[pos[k]] = this->tree_nodes_[i + 1][next_layer_index + k].weight_;
				}
			}
			// last layer subnode weight is 1
			else {
				for (size_t k = 0; k < pos.size(); k++) {
					weight[pos[k]] = 1;
				}
			}

			// coffs number is 1-bits number - 1
			for (size_t k = 0; k < pos.size() - 1; k++) {
				this->tree_nodes_[i][j].cof_y_.emplace_back(coffs_y[coffs_scan_index]);
				this->tree_nodes_[i][j].cof_u_.emplace_back(coffs_u[coffs_scan_index]);
				this->tree_nodes_[i][j].cof_v_.emplace_back(coffs_v[coffs_scan_index]);
				coffs_scan_index++;
			}

			// subnodes signals
			std::vector<float> gys(8, 0.0f), gus(8, 0.0f), gvs(8, 0.0f);

			vvs::operation::NodeSignalIMerge3D(gys, gus, gvs, weight, this->tree_nodes_[i][j]);

			// if not real point
			if (i != this->tree_height_ - 1) {
				for (size_t k = 0; k < pos.size(); k++) {
					this->tree_nodes_[i + 1][next_layer_index].sig_y_ = gys[pos[k]];
					this->tree_nodes_[i + 1][next_layer_index].sig_u_ = gus[pos[k]];
					this->tree_nodes_[i + 1][next_layer_index].sig_v_ = gvs[pos[k]];

					next_layer_index++;
				}
			}
			else {
				for (size_t k = 0; k < pos.size(); k++) {
					vvs::type::ColorYUV color;
					color.y_ = gys[pos[k]], color.u_ = gus[pos[k]], color.v_ = gvs[pos[k]];
					__result.emplace_back(color);
				}
			}
		}
	}
}

/***
 * @description: get patch geometry information
 * @param {PointCloud<PointXYZ>&} __patch
 * @return {*}
 */
void DeOctree3D::GetPatch(std::string& __color_source, pcl::PointCloud<pcl::PointXYZ>& __patch, std::vector<vvs::type::ColorYUV>& __colors, size_t index) {
	// allocate some space
	__patch.resize(this->tree_nodes_.back().size() * 8);

	// how many points
	size_t point_cnt = 0;

	// for each leaf node
	for (size_t i = 0; i < this->tree_nodes_.back().size(); i++) {
		// 1-bit pos
		std::vector<size_t> pos;
		vvs::operation::NodePointPosition(this->tree_nodes_.back()[i].subnodes_, pos);

		// for each 1-bit
		for (auto& k : pos) {
			// calculate point
			pcl::PointXYZ point;
			vvs::operation::SubnodePoint(this->tree_points_.back()[i], k, __patch[point_cnt]);
			point_cnt++;
		}
	}

	// delete excess space
	__patch.resize(point_cnt);

	this->IRAHT(__color_source, __patch.size(), __colors, index);
}