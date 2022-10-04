/***
 * @Author: ChenRP07
 * @Date: 2022-06-30 14:41:50
 * @LastEditTime: 2022-06-30 14:41:51
 * @LastEditors: ChenRP07
 * @Description: Header and Implement of some types.
 */
#ifndef _LIB_TYPE_HPP_
#define _LIB_TYPE_HPP_
#include "dependency/operation.hpp"
#include "dependency/turbojpeg.h"
#include <Eigen/Dense>
#include <cfloat>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
namespace vvs {
namespace type {
	// zigzag in [1]
	// ZigZagI3D8[key] = value, key is zig-zag index, value is morton index
	static size_t ZigZagI3D8[512] = {
	    0,   1,   2,   4,   32,  6,   5,   16,  3,   8,   9,   10,  17,  18,  12,  7,   20,  33,  34,  36,  256, 38,  37,  48,  35,  40,  22,  21,  14,  13,  128, 19,  24,  11,  64,  65,  66,
	    25,  26,  129, 130, 68,  15,  28,  23,  132, 41,  42,  49,  50,  44,  39,  52,  257, 258, 260, 288, 262, 261, 272, 259, 264, 54,  53,  46,  45,  160, 51,  56,  43,  96,  134, 133, 30,
	    29,  70,  69,  144, 131, 136, 27,  80,  67,  72,  73,  74,  81,  82,  137, 138, 145, 146, 76,  71,  84,  31,  140, 135, 148, 97,  98,  57,  58,  161, 162, 100, 47,  60,  55,  164, 265,
	    266, 273, 274, 268, 263, 276, 289, 290, 292, 294, 293, 304, 291, 296, 278, 277, 270, 269, 384, 275, 280, 267, 320, 166, 165, 62,  61,  102, 101, 176, 163, 168, 59,  112, 99,  104, 150,
	    149, 142, 141, 86,  85,  78,  77,  147, 152, 139, 192, 83,  88,  75,  89,  90,  193, 194, 153, 154, 79,  92,  87,  196, 143, 156, 151, 105, 106, 113, 114, 169, 170, 177, 178, 108, 103,
	    116, 63,  172, 167, 180, 321, 322, 281, 282, 385, 386, 324, 271, 284, 279, 388, 297, 298, 305, 306, 300, 295, 308, 310, 309, 302, 301, 416, 307, 312, 299, 352, 390, 389, 286, 285, 326,
	    325, 400, 387, 392, 283, 336, 323, 328, 182, 181, 174, 173, 118, 117, 110, 109, 179, 184, 171, 224, 115, 120, 107, 158, 157, 198, 197, 94,  93,  155, 208, 195, 200, 91,  201, 202, 209,
	    210, 95,  204, 199, 212, 159, 121, 122, 225, 226, 185, 186, 111, 124, 119, 228, 175, 188, 183, 329, 330, 337, 338, 393, 394, 401, 402, 332, 327, 340, 287, 396, 391, 404, 353, 354, 313,
	    314, 417, 418, 356, 303, 316, 311, 420, 422, 421, 318, 317, 358, 357, 432, 419, 424, 315, 368, 355, 360, 406, 405, 398, 397, 342, 341, 334, 333, 403, 408, 395, 448, 339, 344, 331, 190,
	    189, 230, 229, 126, 125, 187, 240, 227, 232, 123, 214, 213, 206, 205, 211, 216, 203, 217, 218, 207, 220, 215, 233, 234, 241, 242, 127, 236, 231, 244, 191, 345, 346, 449, 450, 409, 410,
	    335, 348, 343, 452, 399, 412, 407, 361, 362, 369, 370, 425, 426, 433, 434, 364, 359, 372, 319, 428, 423, 436, 438, 437, 430, 429, 374, 373, 366, 365, 435, 440, 427, 480, 371, 376, 363,
	    414, 413, 454, 453, 350, 349, 411, 464, 451, 456, 347, 246, 245, 238, 237, 243, 248, 235, 222, 221, 219, 223, 249, 250, 239, 252, 247, 457, 458, 465, 466, 351, 460, 455, 468, 415, 377,
	    378, 481, 482, 441, 442, 367, 380, 375, 484, 431, 444, 439, 446, 445, 486, 485, 382, 381, 443, 496, 483, 488, 379, 470, 469, 462, 461, 467, 472, 459, 254, 253, 251, 255, 473, 474, 463,
	    476, 471, 489, 490, 497, 498, 383, 492, 487, 500, 447, 502, 501, 494, 493, 499, 504, 491, 478, 477, 475, 479, 505, 506, 495, 508, 503, 510, 509, 507, 511};

	// ZigZag3D8[key] = value, key is morton index, value is zigzag index
	static size_t ZigZag3D8[512] = {
	    0,   1,   2,   8,   3,   6,   5,   15,  9,   10,  11,  33,  14,  29,  28,  42,  7,   12,  13,  31,  16,  27,  26,  44,  32,  37,  38,  80,  43,  74,  73,  95,  4,   17,  18,  24,  19,
	    22,  21,  51,  25,  46,  47,  69,  50,  65,  64,  106, 23,  48,  49,  67,  52,  63,  62,  108, 68,  101, 102, 143, 107, 137, 136, 186, 34,  35,  36,  82,  41,  76,  75,  93,  83,  84,
	    85,  161, 92,  154, 153, 168, 81,  86,  87,  159, 94,  152, 151, 170, 160, 162, 163, 255, 169, 250, 249, 260, 70,  99,  100, 145, 105, 139, 138, 184, 146, 175, 176, 244, 183, 237, 236,
	    271, 144, 177, 178, 242, 185, 235, 234, 273, 243, 265, 266, 342, 272, 337, 336, 359, 30,  39,  40,  78,  45,  72,  71,  97,  79,  88,  89,  157, 96,  150, 149, 172, 77,  90,  91,  155,
	    98,  148, 147, 174, 156, 166, 167, 251, 173, 246, 245, 264, 66,  103, 104, 141, 109, 135, 134, 188, 142, 179, 180, 240, 187, 233, 232, 275, 140, 181, 182, 238, 189, 231, 230, 277, 239,
	    269, 270, 338, 276, 333, 332, 363, 158, 164, 165, 253, 171, 248, 247, 262, 254, 256, 257, 349, 261, 346, 345, 352, 252, 258, 259, 347, 263, 344, 343, 354, 348, 350, 351, 427, 353, 426,
	    425, 428, 241, 267, 268, 340, 274, 335, 334, 361, 341, 355, 356, 424, 360, 421, 420, 431, 339, 357, 358, 422, 362, 419, 418, 433, 423, 429, 430, 476, 432, 475, 474, 477, 20,  53,  54,
	    60,  55,  58,  57,  115, 61,  110, 111, 132, 114, 128, 127, 197, 59,  112, 113, 130, 116, 126, 125, 199, 131, 192, 193, 226, 198, 220, 219, 289, 56,  117, 118, 123, 119, 121, 120, 206,
	    124, 201, 202, 215, 205, 211, 210, 300, 122, 203, 204, 213, 207, 209, 208, 302, 214, 295, 296, 313, 301, 307, 306, 388, 133, 190, 191, 228, 196, 222, 221, 287, 229, 278, 279, 331, 286,
	    324, 323, 370, 227, 280, 281, 329, 288, 322, 321, 372, 330, 364, 365, 417, 371, 412, 411, 438, 216, 293, 294, 315, 299, 309, 308, 386, 316, 377, 378, 406, 385, 399, 398, 449, 314, 379,
	    380, 404, 387, 397, 396, 451, 405, 443, 444, 466, 450, 461, 460, 487, 129, 194, 195, 224, 200, 218, 217, 291, 225, 282, 283, 327, 290, 320, 319, 374, 223, 284, 285, 325, 292, 318, 317,
	    376, 326, 368, 369, 413, 375, 408, 407, 442, 212, 297, 298, 311, 303, 305, 304, 390, 312, 381, 382, 402, 389, 395, 394, 453, 310, 383, 384, 400, 391, 393, 392, 455, 401, 447, 448, 462,
	    454, 457, 456, 491, 328, 366, 367, 415, 373, 410, 409, 440, 416, 434, 435, 473, 439, 470, 469, 480, 414, 436, 437, 471, 441, 468, 467, 482, 472, 478, 479, 501, 481, 500, 499, 502, 403,
	    445, 446, 464, 452, 459, 458, 489, 465, 483, 484, 498, 488, 495, 494, 505, 463, 485, 486, 496, 490, 493, 492, 507, 497, 503, 504, 510, 506, 509, 508, 511};

	// ZigZagI3D4[key] = value, key is zig-zag index, value is morton index
	static size_t ZigZagI3D4[64] = {0,  1,  2,  4,  32, 6,  5,  16, 3,  8,  9,  10, 17, 18, 12, 7,  20, 33, 34, 36, 38, 37, 48, 35, 40, 22, 21, 14, 13, 19, 24, 11,
	                                25, 26, 15, 28, 23, 41, 42, 49, 50, 44, 39, 52, 54, 53, 46, 45, 51, 56, 43, 30, 29, 27, 31, 57, 58, 47, 60, 55, 62, 61, 59, 63};
	// ZigZag3D8[key] = value, key is morton index, value is zigzag index
	static size_t ZigZag3D4[64] = {0, 1,  2,  8,  3,  6,  5,  15, 9,  10, 11, 31, 14, 28, 27, 34, 7,  12, 13, 29, 16, 26, 25, 36, 30, 32, 33, 53, 35, 52, 51, 54,
	                               4, 17, 18, 23, 19, 21, 20, 42, 24, 37, 38, 50, 41, 47, 46, 57, 22, 39, 40, 48, 43, 45, 44, 59, 49, 55, 56, 62, 58, 61, 60, 63};

	// quantization parameter, see in [2]
	static int QuantizationStep[22] = {16, 12, 12, 14, 17, 26, 34, 55, 64, 80, 85, 90, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
	// static int QuantizationStep[22] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	// static int QuantizationStep[22] = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
	struct ColorRGB;
	struct ColorYUV;
	struct MacroBlock8;
	struct TreeNode;

	struct range {
		float maxx, maxy, maxz;
		float minx, miny, minz;
	};

	// a compressed i-frame patch
	struct IFramePatch {
		// fitting patch octree
		std::string   octree_;
		pcl::PointXYZ center_;
		size_t        tree_height_;
		// fitting colors
		std::string colors_;
		size_t      size;

		size_t total_size;
		range  box;
	};

	// a compressed p-frame patch
	struct PFramePatch {
		// coding mode tag
		bool is_independent_;
		// compressed octree, only be viable if is_independent_ is true
		// fitting patch octree
		std::string   octree_;
		pcl::PointXYZ center_;
		size_t        tree_height_;
		// compressed colors
		std::string colors_;
		// motion vector
		Eigen::Matrix4f motion_vector_;

		size_t total_size;
		range  box;

		/***
		 * @description: clear octree_ and colors_
		 * @param {*}
		 * @return {*}
		 */
		void clear() {
			this->octree_.clear();
			this->colors_.clear();
		}
	};

	// RGB color
	struct ColorRGB {
		float r_, g_, b_;

		/***
		 * @description: default constructor
		 * @param {*}
		 * @return {*}
		 */
		ColorRGB() {
			this->r_ = this->g_ = this->b_ = 0.0f;
		};

		/***
		 * @description: copy constructor from a PointXYZRGB
		 * @param {PointXYZRGB&} __point
		 * @return {*}
		 */
		ColorRGB(const pcl::PointXYZRGB& __point) {
			this->r_ = __point.r, this->g_ = __point.g, this->b_ = __point.b;
		}

		/***
		 * @description: copy constructor
		 * @param {ColorRGB&} __x
		 * @return {*}
		 */
		ColorRGB(const ColorRGB& __x) {
			this->r_ = __x.r_, this->g_ = __x.g_, this->b_ = __x.b_;
		}

		/***
		 * @description: constructor from yuv
		 * @param {ColorYUV&} __x
		 * @return {*}
		 */
		ColorRGB(const ColorYUV& __x);

		/***
		 * @description: interpolation by PointXYZRGB, this += __w * __point
		 * @param {float} __w
		 * @param {PointXYZRGB&} __point
		 * @return {*}
		 */
		void operator()(float __w, pcl::PointXYZRGB& __point) {
			this->r_ += __w * static_cast<float>(__point.r), this->g_ += __w * static_cast<float>(__point.g), this->b_ += __w * static_cast<float>(__point.b);
		}

		/***
		 * @description: interpolation by ColorRGB, this += __x
		 * @param {ColorRGB&} __x
		 * @return {*}
		 */
		ColorRGB& operator+=(const ColorRGB& __x) {
			this->r_ += __x.r_, this->g_ += __x.g_, this->b_ += __x.b_;
			return *this;
		}

		/***
		 * @description: each component /= __x
		 * @param {*}
		 * @return {*}
		 */
		ColorRGB& operator/=(const size_t __x) {
			this->r_ /= static_cast<float>(__x), this->g_ /= static_cast<float>(__x), this->b_ /= static_cast<float>(__x);
			return *this;
		}

		/***
		 * @description: assignment construcotr
		 * @param {*}
		 * @return {*}
		 */
		ColorRGB& operator=(const ColorRGB& __x) {
			this->r_ = __x.r_, this->g_ = __x.g_, this->b_ = __x.b_;
			return *this;
		}
	};

	// color, YUV
	struct ColorYUV {
		float y_, u_, v_;

		ColorYUV() = default;

		/***
		 * @description: copy constructor from rgb
		 * @param {ColorRGB&} __x
		 * @return {*}
		 */
		ColorYUV(const ColorRGB& __x);

		/***
		 * @description: compensation with a TreeNode
		 * @param {TreeNode&} __node
		 * @return {*}
		 */
		ColorYUV& operator-=(const TreeNode& __node);

		/***
		 * @description: compensation with another yuv
		 * @param {ColorYUV&} __x
		 * @return {*}
		 */
		ColorYUV& operator-=(const ColorYUV& __x) {
			this->y_ -= __x.y_;
			this->u_ -= __x.u_;
			this->v_ -= __x.v_;
			return *this;
		}

		/***
		 * @description: inverse compensation with a TreeNode
		 * @param {TreeNode&} __node
		 * @return {*}
		 */
		ColorYUV& operator+=(const TreeNode& __node);

		/***
		 * @description: inverse compensation with another yuv
		 * @param {*}
		 * @return {*}
		 */
		ColorYUV& operator+=(const ColorYUV& __x) {
			this->y_ += __x.y_;
			this->u_ += __x.u_;
			this->v_ += __x.v_;
			return *this;
		}
	};

	// node structure for octree
	struct TreeNode {
		// subnode occupation
		uint8_t subnodes_;
		// signal of YUV
		float sig_y_, sig_u_, sig_v_;
		// coffecients of YUV
		std::vector<float> cof_y_, cof_u_, cof_v_;
		// weight, how many points in this node
		size_t weight_;

		std::vector<size_t> cof_weights_;

		/***
		 * @description: default constructor
		 * @param {*}
		 * @return {*}
		 */
		TreeNode() {
			this->weight_   = 0;
			this->subnodes_ = 0x00;
			this->sig_y_ = this->sig_u_ = this->sig_v_ = 0.0f;
		}

		/***
		 * @description: constructor, init the node weight
		 * @param {size_t} __w
		 * @return {*}
		 */
		TreeNode(size_t __w) {
			this->weight_   = __w;
			this->subnodes_ = 0x00;
			this->sig_y_ = this->sig_u_ = this->sig_v_ = 0.0f;
		}

		/***
		 * @description: constructor, init the octree node occupation
		 * @param {uint8_t} __v
		 * @return {*}
		 */
		TreeNode(uint8_t __v) {
			this->subnodes_ = __v;
			this->weight_   = 0;
			this->sig_y_ = this->sig_u_ = this->sig_v_ = 0.0f;
		}

		/***
		 * @description: duplicate constructor
		 * @param {const TreeNode&} __x
		 * @return {*}
		 */
		TreeNode(const TreeNode& __x) {
			this->subnodes_ = __x.subnodes_, this->weight_ = __x.weight_;
			this->sig_y_ = __x.sig_y_, this->sig_u_ = __x.sig_u_, this->sig_v_ = __x.sig_v_;
			this->cof_y_ = __x.cof_y_, this->cof_u_ = __x.cof_u_, this->cof_v_ = __x.cof_v_;
		}

		/***
		 * @description: assign constructor
		 * @param {const TreeNode&} __x
		 * @return {*}
		 */
		TreeNode& operator=(const TreeNode& __x) {
			this->subnodes_ = __x.subnodes_, this->weight_ = __x.weight_;
			this->sig_y_ = __x.sig_y_, this->sig_u_ = __x.sig_u_, this->sig_v_ = __x.sig_v_;
			this->cof_y_ = __x.cof_y_, this->cof_u_ = __x.cof_u_, this->cof_v_ = __x.cof_v_;
			return *this;
		}

		/***
		 * @description: set subnodes_
		 * @param {uint8_t} __x
		 * @return {*}
		 */
		inline void SetNode(uint8_t __x) {
			this->subnodes_ = __x;
		}

		/***
		 * @description: set weight_
		 * @param {size_t} __w
		 * @return {*}
		 */
		inline void SetWeight(size_t __w) {
			this->weight_ = __w;
		}

		/***
		 * @description: set yuv signal by a RGB
		 * @param {const ColorRGB&} __color
		 * @return {*}
		 */
		inline void SetSignal(const vvs::type::ColorRGB& __color);
	};

	// A 8*8*8 block cube
	struct MacroBlock8 {
		// YUV
		std::vector<ColorYUV> points_;

		MacroBlock8() = default;

		/***
		 * @description: Output block size
		 * @param {*}
		 * @return {*}
		 */
		size_t size() const {
			return this->points_.size();
		}

		/***
		 * @description: if this block is full, i.e., size is 512
		 * @param {*}
		 * @return {*}
		 */
		bool full() const {
			return this->size() == 512;
		}

		/***
		 * @description: if this block is empty
		 * @param {*}
		 * @return {*}
		 */
		bool empty() const {
			return this->size() == 0;
		}

		/***
		 * @description: Add a real point to block
		 * @param {ColorRGB&} __color
		 * @return {*}
		 */
		inline void PushBack(const ColorYUV& __color);

		/***
		 * @description: using 000 fill block to 512
		 * @param {*}
		 * @return {*}
		 */
		inline void Fill();

		/***
		 * @description: 3d-dct->quantization->zigzag-scan
		 * @param {vector<int>&} __coefficients_y
		 * @param {vector<int>&} __coefficients_u
		 * @param {vector<int>&} __coefficients_v
		 * @return {*}
		 */
		inline void YUVDCT3(std::vector<int>& __coefficients_y, std::vector<int>& __coefficients_u, std::vector<int>& __coefficients_v);

		/***
		 * @description: izigzag-scan-dequantization-3d-idct
		 * @param {vector<int>&} __coefficients_y
		 * @param {vector<int>&} __coefficients_u
		 * @param {vector<int>&} __coefficients_v
		 * @return {*}
		 */
		inline void YUVIDCT3(std::vector<int>& __coefficients_r, std::vector<int>& __coefficients_g, std::vector<int>& __coefficients_b);
	};

	/***
	 * @description: copy constructor from yuv
	 * @param {ColorYUV&} __x
	 * @return {*}
	 */
	inline ColorRGB::ColorRGB(const ColorYUV& __x) {
		this->r_ = 1.0f * __x.y_ + 1.4020f * __x.v_;
		this->g_ = 1.0f * __x.y_ - 0.3441f * __x.u_ - 0.7141f * __x.v_;
		this->b_ = 1.0f * __x.y_ + 1.7720f * __x.u_;
	}

	/***
	 * @description: copy constructor from rgb
	 * @param {ColorRGB&} __x
	 * @return {*}
	 */
	inline ColorYUV::ColorYUV(const ColorRGB& __x) {
		this->y_ = 0.299f * __x.r_ + 0.587f * __x.g_ + 0.114f * __x.b_;
		this->u_ = -0.168736f * __x.r_ - 0.331264f * __x.g_ + 0.5f * __x.b_;
		this->v_ = 0.5f * __x.r_ - 0.418688f * __x.g_ - 0.081312f * __x.b_;
	}

	/***
	 * @description: compensation with a TreeNode
	 * @param {TreeNode&} __node
	 * @return {*}
	 */
	inline ColorYUV& ColorYUV::operator-=(const TreeNode& __node) {
		this->y_ -= __node.sig_y_;
		this->u_ -= __node.sig_u_;
		this->v_ -= __node.sig_v_;
		return *this;
	}

	/***
	 * @description: inverse compensation with a TreeNode
	 * @param {TreeNode&} __node
	 * @return {*}
	 */
	inline ColorYUV& ColorYUV::operator+=(const TreeNode& __node) {
		this->y_ += __node.sig_y_;
		this->u_ += __node.sig_u_;
		this->v_ += __node.sig_v_;
		return *this;
	}

	/***
	 * @description: set yuv signal by a RGB
	 * @param {const ColorRGB&} __color
	 * @return {*}
	 */
	inline void TreeNode::SetSignal(const vvs::type::ColorRGB& __color) {
		this->sig_y_ = __color.r_ * 0.299f + __color.g_ * 0.587f + __color.b_ * 0.114f;
		this->sig_u_ = __color.r_ * -0.168736f - __color.g_ * 0.331264f + __color.b_ * 0.5f;
		this->sig_v_ = __color.r_ * 0.5f - __color.g_ * 0.418688f - __color.b_ * 0.081312f;
	}

	/***
	 * @description: Add a real point to block
	 * @param {ColorRGB&} __color
	 * @return {*}
	 */
	inline void MacroBlock8::PushBack(const ColorYUV& __color) {
		try {
			if (!this->full()) {
				this->points_.emplace_back(__color);
			}
			else {
				throw "Block is full.";
			}
		}
		catch (const char* error_message) {
			std::cerr << "Fatal errro in MacroBlock8 PushBack : " << error_message << std::endl;
			std::exit(1);
		}
	}

	/***
	 * @description: using 000 fill block to 512
	 * @param {*}
	 * @return {*}
	 */
	inline void MacroBlock8::Fill() {
		while (!this->full()) {
			this->PushBack(ColorYUV());
		}
	}

	/***
	 * @description: 3d-dct->quantization->zigzag-scan
	 * @param {vector<int>&} __coefficients_y
	 * @param {vector<int>&} __coefficients_u
	 * @param {vector<int>&} __coefficients_v
	 * @return {*}
	 */
	inline void MacroBlock8::YUVDCT3(std::vector<int>& __coefficients_y, std::vector<int>& __coefficients_u, std::vector<int>& __coefficients_v) {
		try {
			if (!this->full()) {
				throw "MacroBlock8 is not full.";
			}
			// reserve space, 512 for y u v
			__coefficients_y.clear(), __coefficients_u.clear(), __coefficients_v.clear();
			__coefficients_y.resize(512), __coefficients_u.resize(512), __coefficients_v.resize(512);

			// translate y to -128-128
			for (auto& j : this->points_) {
				j.y_ -= 128.0f;
			}
			// for each yuv
			for (int k1 = 0; k1 < 8; k1++) {
				for (int k2 = 0; k2 < 8; k2++) {
					for (int k3 = 0; k3 < 8; k3++) {
						// sum each component
						float sum_y = 0.0f, sum_u = 0.0f, sum_v = 0.0f;
						for (int n1 = 0; n1 < 8; n1++) {
							for (int n2 = 0; n2 < 8; n2++) {
								for (int n3 = 0; n3 < 8; n3++) {
									// turn xyz to morton index
									size_t index = vvs::operation::XYZ2Index8(n1, n2, n3);
									// cos transform, in [1]
									sum_y += this->points_[index].y_ * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
									sum_u += this->points_[index].u_ * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
									sum_v += this->points_[index].v_ * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
								}
							}
						}
						// turn xyz to morton index
						size_t yuv_index = vvs::operation::XYZ2Index8(k1, k2, k3);

						// orth and zigzag in [1], quantization in [2]
						__coefficients_y[ZigZag3D8[yuv_index]] =
						    static_cast<int>(std::round((sum_y * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
						__coefficients_u[ZigZag3D8[yuv_index]] =
						    static_cast<int>(std::round((sum_u * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
						__coefficients_v[ZigZag3D8[yuv_index]] =
						    static_cast<int>(std::round((sum_v * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
					}
				}
			}
		}
		catch (const char* error_message) {
			std::cerr << "Fatal error in MacroBlock8 YUVDCT3 : " << error_message << std::endl;
			std::exit(1);
		}
	}

	/***
	 * @description: izigzag-scan-dequantization-3d-idct
	 * @param {vector<int>&} __coefficients_y
	 * @param {vector<int>&} __coefficients_u
	 * @param {vector<int>&} __coefficients_v
	 * @return {*}
	 */
	inline void MacroBlock8::YUVIDCT3(std::vector<int>& __coefficients_y, std::vector<int>& __coefficients_u, std::vector<int>& __coefficients_v) {
		this->points_.clear();
		this->points_.resize(512);
		for (int k1 = 0; k1 < 8; k1++) {
			for (int k2 = 0; k2 < 8; k2++) {
				for (int k3 = 0; k3 < 8; k3++) {
					size_t index = vvs::operation::XYZ2Index8(k1, k2, k3);
					__coefficients_y[index] *= QuantizationStep[k1 + k2 + k3];
					__coefficients_u[index] *= QuantizationStep[k1 + k2 + k3];
					__coefficients_v[index] *= QuantizationStep[k1 + k2 + k3];
				}
			}
		}

		// for each yuv
		for (int n1 = 0; n1 < 8; n1++) {
			for (int n2 = 0; n2 < 8; n2++) {
				for (int n3 = 0; n3 < 8; n3++) {
					// sum all components
					float sum_y = 0.0f, sum_u = 0.0f, sum_v = 0.0f;
					for (int k1 = 0; k1 < 8; k1++) {
						for (int k2 = 0; k2 < 8; k2++) {
							for (int k3 = 0; k3 < 8; k3++) {
								// morton index
								size_t index = vvs::operation::XYZ2Index8(k1, k2, k3);
								sum_y += vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) * __coefficients_y[ZigZag3D8[index]] *
								         std::cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1) * k1) * std::cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1) * k2) * std::cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1) * k3);
								sum_u += vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) * __coefficients_u[ZigZag3D8[index]] *
								         std::cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1) * k1) * std::cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1) * k2) * std::cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1) * k3);
								sum_v += vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) * __coefficients_v[ZigZag3D8[index]] *
								         std::cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1) * k1) * std::cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1) * k2) * std::cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1) * k3);
							}
						}
					}
					// turn xyz to morton index
					size_t yuv_index = vvs::operation::XYZ2Index8(n1, n2, n3);

					this->points_[yuv_index].y_ = sum_y + 128, this->points_[yuv_index].u_ = sum_u, this->points_[yuv_index].v_ = sum_v;
				}
			}
		}
	}
}  // namespace type

namespace operation {
	inline void JPEGCompression(std::vector<vvs::type::ColorRGB>& __colors, std::string& __result) {
		size_t image_height = static_cast<int>(std::sqrt(__colors.size()));
		size_t image_width  = __colors.size() / image_height;

		if (image_width * image_height != __colors.size()) {
			image_width++;
		}

		unsigned char  buffer[image_height * image_width * 3];
		unsigned char* compressed_image = nullptr;

		size_t index = 0;
		for (size_t i = 0; i < __colors.size(); i++) {
			buffer[index]     = __colors[i].r_;
			buffer[index + 1] = __colors[i].g_;
			buffer[index + 2] = __colors[i].b_;
			index += 3;
		}

		while (index < image_height * image_width * 3) {
			buffer[index]     = __colors.back().r_;
			buffer[index + 1] = __colors.back().g_;
			buffer[index + 2] = __colors.back().b_;
			index += 3;
		}

		long unsigned int jpeg_size = 0;

		tjhandle jpeg_compressor = tjInitCompress();

		tjCompress2(jpeg_compressor, buffer, image_width, 0, image_height, TJPF_RGB, &compressed_image, &jpeg_size, TJSAMP_420, 90, TJFLAG_FASTDCT);

		for (size_t i = 0; i < jpeg_size; i++) {
			__result += static_cast<char>(compressed_image[i]);
		}
	}

	inline void JPEGDecompression(std::vector<vvs::type::ColorRGB>& __colors, std::string& __source) {
		unsigned char compressed_image[__source.size()];
		for (size_t i = 0; i < __source.size(); i++) {
			compressed_image[i] = static_cast<unsigned char>(__source[i]);
		}

		tjhandle jpeg_decompressor = tjInitDecompress();
		int      image_width, image_height, jpeg_subsamp;
		tjDecompressHeader2(jpeg_decompressor, compressed_image, __source.size(), &image_width, &image_height, &jpeg_subsamp);

		unsigned char buffer[image_height * image_width * 3];

		tjDecompress2(jpeg_decompressor, compressed_image, __source.size(), buffer, image_width, 0, image_height, TJPF_RGB, TJFLAG_FASTDCT);

		__colors.resize(image_width * image_height);
		for (size_t i = 0; i < image_width * image_height; i++) {
			__colors[i].r_ = static_cast<float>(buffer[i * 3]);
			__colors[i].g_ = static_cast<float>(buffer[i * 3 + 1]);
			__colors[i].b_ = static_cast<float>(buffer[i * 3 + 2]);
		}
	}

	inline Eigen::Vector3f RAHTransform(float g0, float g1, size_t w0, size_t w1) {
		// result : MergeSignal MergeCoffecients CofExist
		Eigen::Vector3f result;
		// no weight is positive, result(2) < 0, i.e., no coffecient
		if (w0 == 0 && w1 == 0) {
			result(0) = 0.0f;
			result(2) = -1.0f;
		}
		// if only w0 is 0, MergeSignal is g1, no coffecient
		else if (w0 == 0) {
			result(0) = g1;
			result(2) = -1.0f;
		}
		// if only w1 is 0, MergeSignal is g0, no coffecient
		else if (w1 == 0) {
			result(0) = g0;
			result(2) = -1.0f;
		}
		// both w0 w1 are positive,
		// [ g ] = [ √w0 √w1 ]
		// [ h ]   [-√w1 √w0 ] / √(w0 + w1)
		else {
			float q0 = std::sqrt(w0), q1 = std::sqrt(w1);
			float qs  = std::sqrt(w0 + w1);
			result(0) = (q0 * g0 + q1 * g1) / qs;
			result(1) = (q0 * g1 - q1 * g0) / qs;
			result(2) = 1.0f;
		}

		return result;
	}

	inline void NodeSignalMerge3D(std::vector<float>& gys, std::vector<float>& gus, std::vector<float>& gvs, std::vector<size_t>& weight, vvs::type::TreeNode& node) {
		// x-direction
		std::vector<float>  xgys(4, 0.0f), xgus(4, 0.0f), xgvs(4, 0.0f);
		std::vector<float>  xcofys, xcofus, xcofvs;
		std::vector<size_t> xweights(4, 0);

		for (size_t i = 0; i < 4; i++) {
			Eigen::Vector3f resulty = RAHTransform(gys[i], gys[i + 4], weight[i], weight[i + 4]);
			if (resulty(2) > 0.0f) {
				xcofys.emplace_back(resulty(1));
			}
			xgys[i] = resulty(0);

			Eigen::Vector3f resultu = RAHTransform(gus[i], gus[i + 4], weight[i], weight[i + 4]);
			if (resultu(2) > 0.0f) {
				xcofus.emplace_back(resultu(1));
			}
			xgus[i] = resultu(0);

			Eigen::Vector3f resultv = RAHTransform(gvs[i], gvs[i + 4], weight[i], weight[i + 4]);
			if (resultv(2) > 0.0f) {
				xcofvs.emplace_back(resultv(1));
			}
			xgvs[i] = resultv(0);

			xweights[i] = weight[i] + weight[i + 4];
		}

		// y-direction
		std::vector<float>  ygys(2, 0.0f), ygus(2, 0.0f), ygvs(2, 0.0f);
		std::vector<float>  ycofys, ycofus, ycofvs;
		std::vector<size_t> yweights(2, 0);

		for (size_t i = 0; i < 2; i++) {
			Eigen::Vector3f resulty = RAHTransform(xgys[i], xgys[i + 2], xweights[i], xweights[i + 2]);
			if (resulty(2) > 0.0f) {
				ycofys.emplace_back(resulty(1));
			}
			ygys[i] = resulty(0);

			Eigen::Vector3f resultu = RAHTransform(xgus[i], xgus[i + 2], xweights[i], xweights[i + 2]);
			if (resultu(2) > 0.0f) {
				ycofus.emplace_back(resultu(1));
			}
			ygus[i] = resultu(0);

			Eigen::Vector3f resultv = RAHTransform(xgvs[i], xgvs[i + 2], xweights[i], xweights[i + 2]);
			if (resultv(2) > 0.0f) {
				ycofvs.emplace_back(resultv(1));
			}
			ygvs[i] = resultv(0);

			yweights[i] = xweights[i] + xweights[i + 2];
		}

		// z-direction
		float              zgys = 0.0f, zgus = 0.0f, zgvs = 0.0f;
		std::vector<float> zcofys, zcofus, zcofvs;
		size_t             zweights = 0;

		Eigen::Vector3f resulty = RAHTransform(ygys[0], ygys[1], yweights[0], yweights[1]);
		if (resulty(2) > 0.0f) {
			zcofys.emplace_back(resulty(1));
		}
		zgys = resulty(0);

		Eigen::Vector3f resultu = RAHTransform(ygus[0], ygus[1], yweights[0], yweights[1]);
		if (resultu(2) > 0.0f) {
			zcofus.emplace_back(resultu(1));
		}
		zgus = resultu(0);

		Eigen::Vector3f resultv = RAHTransform(ygvs[0], ygvs[1], yweights[0], yweights[1]);
		if (resultv(2) > 0.0f) {
			zcofvs.emplace_back(resultv(1));
		}
		zgvs = resultv(0);

		zweights = yweights[0] + yweights[1];

		node.sig_y_ = zgys, node.sig_u_ = zgus, node.sig_v_ = zgvs;

		for (auto& i : zcofys) {
			node.cof_y_.emplace_back(i);
		}
		for (auto& i : ycofys) {
			node.cof_y_.emplace_back(i);
		}
		for (auto& i : xcofys) {
			node.cof_y_.emplace_back(i);
		}

		for (auto& i : zcofus) {
			node.cof_u_.emplace_back(i);
		}
		for (auto& i : ycofus) {
			node.cof_u_.emplace_back(i);
		}
		for (auto& i : xcofus) {
			node.cof_u_.emplace_back(i);
		}

		for (auto& i : zcofvs) {
			node.cof_v_.emplace_back(i);
		}
		for (auto& i : ycofvs) {
			node.cof_v_.emplace_back(i);
		}
		for (auto& i : xcofvs) {
			node.cof_v_.emplace_back(i);
		}
	}

	inline Eigen::Vector2f IRAHTransform(float g, float h, size_t w0, size_t w1) {
		Eigen::Vector2f result;

		float a = std::sqrt(w0);
		float b = std::sqrt(w1);
		float s = std::sqrt(w0 + w1);

		g *= s, h *= s;

		result(0) = (a * g - b * h) / (w0 + w1);
		result(1) = (b * g + a * h) / (w0 + w1);

		return result;
	}

	inline void NodeSignalIMerge3D(std::vector<float>& gys, std::vector<float>& gus, std::vector<float>& gvs, std::vector<size_t>& weight, vvs::type::TreeNode& node) {
		size_t cof_index = 0;

		// iz-dimension
		size_t zweight0 = weight[0] + weight[2] + weight[4] + weight[6], zweight1 = weight[1] + weight[3] + weight[5] + weight[7];

		std::vector<float> zgys(2, 0.0f), zgus(2, 0.0f), zgvs(2, 0.0f);

		if (zweight0 == 0) {
			zgys[1] = node.sig_y_;
			zgus[1] = node.sig_u_;
			zgvs[1] = node.sig_v_;
		}
		else if (zweight1 == 0) {
			zgys[0] = node.sig_y_;
			zgus[0] = node.sig_u_;
			zgvs[0] = node.sig_v_;
		}
		else {
			Eigen::Vector2f resulty = IRAHTransform(node.sig_y_, node.cof_y_[cof_index], zweight0, zweight1);
			zgys[0]                 = resulty(0);
			zgys[1]                 = resulty(1);

			Eigen::Vector2f resultu = IRAHTransform(node.sig_u_, node.cof_u_[cof_index], zweight0, zweight1);
			zgus[0]                 = resultu(0);
			zgus[1]                 = resultu(1);

			Eigen::Vector2f resultv = IRAHTransform(node.sig_v_, node.cof_v_[cof_index], zweight0, zweight1);
			zgvs[0]                 = resultv(0);
			zgvs[1]                 = resultv(1);

			cof_index++;
		}

		// iy-dimension
		std::vector<size_t> yweights(4);
		yweights[0] = weight[0] + weight[4], yweights[1] = weight[1] + weight[5], yweights[2] = weight[2] + weight[6], yweights[3] = weight[3] + weight[7];

		std::vector<float> ygys(4, 0.0f), ygus(4, 0.0f), ygvs(4, 0.0f);

		for (size_t i = 0; i < 2; i++) {
			if (yweights[i] == 0) {
				ygys[i + 2] = zgys[i];
				ygus[i + 2] = zgus[i];
				ygvs[i + 2] = zgvs[i];
			}
			else if (yweights[i + 2] == 0) {
				ygys[i] = zgys[i];
				ygus[i] = zgus[i];
				ygvs[i] = zgvs[i];
			}
			else {
				Eigen::Vector2f resulty = IRAHTransform(zgys[i], node.cof_y_[cof_index], yweights[i], yweights[i + 2]);
				ygys[i] = resulty(0), ygys[i + 2] = resulty(1);

				Eigen::Vector2f resultu = IRAHTransform(zgus[i], node.cof_u_[cof_index], yweights[i], yweights[i + 2]);
				ygus[i] = resultu(0), ygus[i + 2] = resultu(1);

				Eigen::Vector2f resultv = IRAHTransform(zgvs[i], node.cof_v_[cof_index], yweights[i], yweights[i + 2]);
				ygvs[i] = resultv(0), ygvs[i + 2] = resultv(1);

				cof_index++;
			}
		}

		// ix-dimension
		for (size_t i = 0; i < 4; i++) {
			if (weight[i] == 0) {
				gys[i + 4] = ygys[i];
				gus[i + 4] = ygus[i];
				gvs[i + 4] = ygvs[i];
			}
			else if (weight[i + 4] == 0) {
				gys[i] = ygys[i];
				gus[i] = ygus[i];
				gvs[i] = ygvs[i];
			}
			else {
				Eigen::Vector2f resulty = IRAHTransform(ygys[i], node.cof_y_[cof_index], weight[i], weight[i + 4]);
				gys[i] = resulty(0), gys[i + 4] = resulty(1);

				Eigen::Vector2f resultu = IRAHTransform(ygus[i], node.cof_u_[cof_index], weight[i], weight[i + 4]);
				gus[i] = resultu(0), gus[i + 4] = resultu(1);

				Eigen::Vector2f resultv = IRAHTransform(ygvs[i], node.cof_v_[cof_index], weight[i], weight[i + 4]);
				gvs[i] = resultv(0), gvs[i + 4] = resultv(1);

				cof_index++;
			}
		}
	}

	inline void CalculateWeights(std::vector<size_t>& weights, vvs::type::TreeNode& node) {
		node.cof_weights_.clear();
		size_t zweight0 = weights[0] + weights[2] + weights[4] + weights[6], zweight1 = weights[1] + weights[3] + weights[5] + weights[7];
		if (zweight0 != 0 && zweight1 != 0) {
			node.cof_weights_.emplace_back(zweight0 + zweight1);
		}

		size_t yweight0 = weights[0] + weights[4], yweight1 = weights[1] + weights[5], yweight2 = weights[2] + weights[6], yweight3 = weights[3] + weights[7];

		if (yweight0 != 0 && yweight2 != 0) {
			node.cof_weights_.emplace_back(yweight0 + yweight2);
		}
		if (yweight1 != 0 && yweight3 != 0) {
			node.cof_weights_.emplace_back(yweight1 + yweight3);
		}

		for (size_t i = 0; i < 4; i++) {
			if (weights[i] != 0 && weights[i + 4] != 0) {
				node.cof_weights_.emplace_back(weights[i] + weights[i + 4]);
			}
		}
	}
}  // namespace operation
}  // namespace vvs

// References
// [1] Said Boussakta et. al. "Fast Algorithm for the 3-D DCT-II" IEEE TRANSACTIONS ON SIGNAL PROCESSING, 2004
// [2] Boon-Lock Yeo et. al. "Volume Rendering of DCT-Based Compressed 3D Scalar Data" IEEE TRANSACTIONS ON VISUALIZATION AND COMPUTER GRAPHICS, 1995

#endif