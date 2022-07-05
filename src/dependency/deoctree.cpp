/***
 * @Author: ChenRP07
 * @Date: 2022-07-04 11:35:23
 * @LastEditTime: 2022-07-04 15:30:46
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
	this->tree_height_ = __height;
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
		this->tree_nodes_.resize(this->tree_height_, std::vector<uint8_t>());

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
				vvs::operation::NodePointPosition(this->tree_nodes_[i][j], pos);

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
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in DeOctree3D SetTree : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get patch geometry information
 * @param {PointCloud<PointXYZ>&} __patch
 * @return {*}
 */
void DeOctree3D::GetPatch(pcl::PointCloud<pcl::PointXYZ>& __patch) {
	// allocate some space
	__patch.resize(this->tree_nodes_.back().size() * 8);

	// how many points
	size_t point_cnt = 0;

	// for each leaf node
	for (size_t i = 0; i < this->tree_nodes_.back().size(); i++) {
		// 1-bit pos
		std::vector<size_t> pos;
		vvs::operation::NodePointPosition(this->tree_nodes_.back()[i], pos);

		// for each 1-bit
		for (auto& k : pos) {
			// calculate point
			vvs::operation::SubnodePoint(this->tree_points_.back()[i], k, __patch[point_cnt]);
			point_cnt++;
		}
	}

	// delete excess space
	__patch.resize(point_cnt);
}