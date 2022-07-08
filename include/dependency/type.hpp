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
	// static int QuantizationStep[22] = {16, 12, 12, 14, 17, 26, 34, 55, 64, 80, 85, 90, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
	static int QuantizationStep[22] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	// point residual
	struct Residual {
		float x_;
		float y_;
		float z_;
		Residual(float __x, float __y, float __z) {
			this->x_ = __x, this->y_ = __y, this->z_ = __z;
		}
	};
	// point colors
	struct Color {
		uint8_t r_;
		uint8_t g_;
		uint8_t b_;
		Color() {
			this->r_ = this->g_ = this->b_ = 0x00;
		}
		Color(pcl::PointXYZRGB& __x) {
			this->r_ = __x.r, this->g_ = __x.g, this->b_ = __x.b;
		}
		Color& operator=(const Color& __x) {
			this->r_ = __x.r_, this->g_ = __x.g_, this->b_ = __x.b_;
			return *this;
		}
		Color& operator+=(const pcl::PointXYZRGB& __x) {
			this->r_ += __x.r, this->g_ += __x.g, this->b_ += __x.b;
			return *this;
		}
		Color& operator-=(const Color& __x) {
			this->r_ -= __x.r_, this->g_ -= __x.g_, this->b_ -= __x.b_;
			return *this;
		}
	};

	// RGB color
	struct ColorRGB {
		float r_;
		float g_;
		float b_;
		ColorRGB() {
			this->r_ = this->g_ = this->b_ = 0.0f;
		}
		ColorRGB(const pcl::PointXYZRGB& __point) {
			this->r_ = __point.r, this->g_ = __point.g, this->b_ = __point.b;
		}
		ColorRGB(const ColorRGB& __x) {
			this->r_ = __x.r_, this->g_ = __x.g_, this->b_ = __x.b_;
		}
		ColorRGB(const float __y, const float __u, const float __v) {
			this->r_ = 1.0f * __y + 1.402 * __v;
			this->g_ = 1.0f * __y - 0.3441 * __u - 0.7141 * __v;
			this->b_ = 1.0f * __y + 1.772 * __u;
		}
		void operator()(float __w, pcl::PointXYZRGB& __point) {
			this->r_ += __w * static_cast<float>(__point.r), this->g_ += __w * static_cast<float>(__point.g), this->b_ += __w * static_cast<float>(__point.b);
		}
		ColorRGB& operator+=(const ColorRGB& __x) {
			this->r_ += __x.r_, this->g_ += __x.g_, this->b_ += __x.b_;
			return *this;
		}
		ColorRGB& operator-=(const ColorRGB& __x) {
			this->r_ -= __x.r_, this->g_ -= __x.g_, this->b_ -= __x.b_;
			return *this;
		}
		ColorRGB& operator/=(const size_t __x) {
			this->r_ /= static_cast<float>(__x), this->g_ /= static_cast<float>(__x), this->b_ /= static_cast<float>(__x);
			return *this;
		}
		ColorRGB& operator=(const ColorRGB& __x) {
			this->r_ = __x.r_, this->g_ = __x.g_, this->b_ = __x.b_;
			return *this;
		}
	};

	// A 8*8*8 block cube
	struct MacroBlock8 {
		// RGB colors
		std::vector<ColorRGB> RGB_;
		// 1 is real point and 0 is virtual point
		std::vector<bool> color_index_;
		// YUV820
		std::vector<float> Y_;
		std::vector<float> U_;
		std::vector<float> V_;

		/***
		 * @description: Output block size
		 * @param {*}
		 * @return {*}
		 */
		size_t size() const {
			try {
				if (RGB_.size() != color_index_.size()) {
					throw "MacroBlock8 is broken.";
				}
				return RGB_.size();
			}
			catch (const char* error_message) {
				std::cerr << "Fatal error : " << error_message << std::endl;
				std::exit(1);
			}
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
		void PushBack(const ColorRGB& __color) {
			try {
				if (this->size() >= 512) {
					throw "Out of block range.";
				}
				this->RGB_.emplace_back(__color);
				this->color_index_.emplace_back(true);
			}
			catch (const char* error_message) {
				std::cerr << "Fatal error : " << error_message << std::endl;
				std::exit(1);
			}
		}

		/***
		 * @description: Add __size virtual empty points to block
		 * @param {size_t} __size
		 * @return {*}
		 */
		void PushBack(const size_t __size) {
			try {
				if (this->size() >= 512) {
					throw "Out of block range.";
				}
				for (size_t i = 0; i < __size; i++) {
					this->RGB_.emplace_back();
					this->color_index_.emplace_back(false);
				}
			}
			catch (const char* error_message) {
				std::cerr << "Fatal error : " << error_message << std::endl;
				std::exit(1);
			}
		}

		void Fill() {
			while (this->size() < 512) {
				this->PushBack(1);
			}
		}
		/***
		 * @description: i-frame filling, using mean real color to fill empty points
		 * @param {*}
		 * @return {*}
		 */
		void IFrameFill() {
			ColorRGB mean_color;
			size_t   count = 0;
			for (size_t i = 0; i < this->size(); i++) {
				if (this->color_index_[i]) {
					mean_color += this->RGB_[i];
					count++;
				}
			}
			mean_color.r_ /= count, mean_color.g_ /= count, mean_color.b_ /= count;
			for (size_t i = 0; i < this->size(); i++) {
				if (!this->color_index_[i]) {
					this->RGB_[i] = mean_color;
				}
			}
		}

		/***
		 * @description: color sampling, all Y reserved, each 8 nodes share a U and a V.
		 * @param {*}
		 * @return {*}
		 */
		void RGB2YUV820() {
			// for each point
			for (size_t i = 0; i < 64; i++) {
				// average rgb for each real point
				float  r = 0.0f, g = 0.0f, b = 0.0f;
				size_t count = 0;
				// for each 8 point
				for (size_t j = 0; j < 8; j++) {
					// y 0.0 for vitural point
					float y = 0.0f;
					// real point
					if (this->color_index_[i * 8 + j]) {
						// convert rgb to y
						y = 0.299f * this->RGB_[i * 8 + j].r_ + 0.587f * this->RGB_[i * 8 + j].g_ + 0.114f * this->RGB_[i * 8 + j].b_;
						r += this->RGB_[i * 8 + j].r_, g += this->RGB_[i * 8 + j].g_, b += this->RGB_[i * 8 + j].b_;
						count++;
					}
					// add y
					this->Y_.emplace_back(y);
				}
				float u = 0.0f, v = 0.0f;
				// at least a real point
				if (count != 0) {
					// mean r g b, conver to u and v
					r /= static_cast<float>(count), g /= static_cast<float>(count), b /= static_cast<float>(count);
					u = -0.168736f * r - 0.331264f * g + 0.5f * b + 128;
					v = 0.5f * r - 0.418688f * g - 0.081312f * b + 128;
				}
				// add u and v
				this->U_.emplace_back(u), this->V_.emplace_back(v);
			}
		}

		/***
		 * @description: color compensation for YUV
		 * @param {const MacroBlock8&} __x
		 * @return {*}
		 */
		MacroBlock8& operator-=(const MacroBlock8& __x) {
			try {
				// if (this->Y_.size() != 512 || this->U_.size() != 64 || this->V_.size() != 64 || __x.Y_.size() != 512 || __x.U_.size() != 64 || __x.V_.size() != 64) {
				// 	throw "Wrong YUV size.";
				// }
				// // sub
				// for (size_t i = 0; i < this->Y_.size(); i++) {
				// 	this->Y_[i] -= __x.Y_[i];
				// }
				// for (size_t i = 0; i < this->U_.size(); i++) {
				// 	this->U_[i] -= __x.U_[i];
				// 	this->V_[i] -= __x.V_[i];
				// }
				for (size_t i = 0; i < this->RGB_.size(); i++) {
					this->RGB_[i] -= __x.RGB_[i];
				}
				return *this;
			}
			catch (const char* error_message) {
				std::cerr << "Fatal error in ColorCompensation : " << error_message << std::endl;
				std::exit(1);
			}
		}

		/***
		 * @description: 3d-dct->quantization->zigzag-scan
		 * @param {vector<int>&} __coefficients_y
		 * @param {vector<int>&} __coefficients_u
		 * @param {vector<int>&} __coefficients_v
		 * @return {*}
		 */
		void YUVDCT3(std::vector<int>& __coefficients_y, std::vector<int>& __coefficients_u, std::vector<int>& __coefficients_v) {
			// reserve space, 512 for y, 64 for u v
			__coefficients_y.clear(), __coefficients_u.clear(), __coefficients_v.clear();
			__coefficients_y.resize(512), __coefficients_u.resize(64), __coefficients_v.resize(64);
			// for each y
			for (int k1 = 0; k1 < 8; k1++) {
				for (int k2 = 0; k2 < 8; k2++) {
					for (int k3 = 0; k3 < 8; k3++) {
						// sum each component
						float sum = 0.0f;
						for (int n1 = 0; n1 < 8; n1++) {
							for (int n2 = 0; n2 < 8; n2++) {
								for (int n3 = 0; n3 < 8; n3++) {
									// turn xyz to morton index
									size_t index = vvs::operation::XYZ2Index8(n1, n2, n3);
									// cos transform, in [1]
									sum += this->Y_[index] * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									       cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
								}
							}
						}
						// turn xyz to morton index
						size_t y_index = vvs::operation::XYZ2Index8(k1, k2, k3);

						// orth and zigzag in [1], quantization in [2]
						__coefficients_y[ZigZag3D8[y_index]] =
						    static_cast<int>(std::round((sum * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
					}
				}
			}

			// for each u v
			for (int k1 = 0; k1 < 4; k1++) {
				for (int k2 = 0; k2 < 4; k2++) {
					for (int k3 = 0; k3 < 4; k3++) {
						// sum each component
						float sum_u = 0.0f, sum_v = 0.0f;
						for (int n1 = 0; n1 < 4; n1++) {
							for (int n2 = 0; n2 < 4; n2++) {
								for (int n3 = 0; n3 < 4; n3++) {
									// turn xyz to morton index
									size_t index = vvs::operation::XYZ2Index4(n1, n2, n3);
									// cos transform in [1]
									sum_u += this->U_[index] * cos(M_PI / (2.0f * 4) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 4) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 4) * (2.0f * n3 + 1.0f) * k3);
									sum_v += this->V_[index] * cos(M_PI / (2.0f * 4) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 4) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 4) * (2.0f * n3 + 1.0f) * k3);
								}
							}
						}

						// turn xyz to morton index
						size_t uv_index = vvs::operation::XYZ2Index4(k1, k2, k3);

						// orth and zigzag in [1], quantization in [2]
						__coefficients_u[ZigZag3D4[uv_index]] =
						    static_cast<int>(std::round(sum_u * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 8.0f) / QuantizationStep[k1 + k2 + k3]);
						__coefficients_v[ZigZag3D4[uv_index]] =
						    static_cast<int>(std::round(sum_v * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 8.0f) / QuantizationStep[k1 + k2 + k3]);
					}
				}
			}
		}

		void RGBDCT3(std::vector<int>& __coefficients_r, std::vector<int>& __coefficients_g, std::vector<int>& __coefficients_b) {
			// reserve space, 512 for y, 64 for u v
			__coefficients_r.clear(), __coefficients_g.clear(), __coefficients_b.clear();
			__coefficients_r.resize(512), __coefficients_g.resize(512), __coefficients_b.resize(512);

			for (size_t i = 0; i < this->RGB_.size(); i++) {
				this->RGB_[i].r_ -= 128, this->RGB_[i].g_ -= 128, this->RGB_[i].b_ -= 128;
			}
			// for each y
			for (int k1 = 0; k1 < 8; k1++) {
				for (int k2 = 0; k2 < 8; k2++) {
					for (int k3 = 0; k3 < 8; k3++) {
						// sum each component
						float sum_r = 0.0f, sum_g = 0.0f, sum_b = 0.0f;
						for (int n1 = 0; n1 < 8; n1++) {
							for (int n2 = 0; n2 < 8; n2++) {
								for (int n3 = 0; n3 < 8; n3++) {
									// turn xyz to morton index
									size_t index = vvs::operation::XYZ2Index8(n1, n2, n3);
									// cos transform, in [1]
									sum_r += this->RGB_[index].r_ * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
									sum_g += this->RGB_[index].g_ * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
									sum_b += this->RGB_[index].b_ * cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1.0f) * k1) * cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1.0f) * k2) *
									         cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1.0f) * k3);
								}
							}
						}
						// turn xyz to morton index
						size_t y_index = vvs::operation::XYZ2Index8(k1, k2, k3);

						// orth and zigzag in [1], quantization in [2]
						__coefficients_r[ZigZag3D8[y_index]] =
						    static_cast<int>(std::round((sum_r * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
						__coefficients_g[ZigZag3D8[y_index]] =
						    static_cast<int>(std::round((sum_g * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
						__coefficients_b[ZigZag3D8[y_index]] =
						    static_cast<int>(std::round((sum_b * vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) / 64.0f) / QuantizationStep[k1 + k2 + k3]));
					}
				}
			}
		}

		void RGBIDCT3(std::vector<int>& __coefficients_r, std::vector<int>& __coefficients_g, std::vector<int>& __coefficients_b) {
			this->RGB_.clear();
			this->RGB_.resize(512);
			for (int k1 = 0; k1 < 8; k1++) {
				for (int k2 = 0; k2 < 8; k2++) {
					for (int k3 = 0; k3 < 8; k3++) {
						size_t index = vvs::operation::XYZ2Index8(k1, k2, k3);
						__coefficients_r[index] *= QuantizationStep[k1 + k2 + k3];
						__coefficients_g[index] *= QuantizationStep[k1 + k2 + k3];
						__coefficients_b[index] *= QuantizationStep[k1 + k2 + k3];

						// __coefficients_r[index] *= 5.0f;
						// __coefficients_g[index] *= 5.0f;
						// __coefficients_b[index] *= 5.0f;
					}
				}
			}
			for (int n1 = 0; n1 < 8; n1++) {
				for (int n2 = 0; n2 < 8; n2++) {
					for (int n3 = 0; n3 < 8; n3++) {
						float sum_r = 0.0f, sum_g = 0.0f, sum_b = 0.0f;
						for (int k1 = 0; k1 < 8; k1++) {
							for (int k2 = 0; k2 < 8; k2++) {
								for (int k3 = 0; k3 < 8; k3++) {
									size_t index = vvs::operation::XYZ2Index8(k1, k2, k3);
									sum_r += vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) * __coefficients_r[ZigZag3D8[index]] *
									         std::cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1) * k1) * std::cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1) * k2) *
									         std::cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1) * k3);
									sum_g += vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) * __coefficients_g[ZigZag3D8[index]] *
									         std::cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1) * k1) * std::cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1) * k2) *
									         std::cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1) * k3);
									sum_b += vvs::operation::Orth(k1) * vvs::operation::Orth(k2) * vvs::operation::Orth(k3) * __coefficients_b[ZigZag3D8[index]] *
									         std::cos(M_PI / (2.0f * 8) * (2.0f * n1 + 1) * k1) * std::cos(M_PI / (2.0f * 8) * (2.0f * n2 + 1) * k2) *
									         std::cos(M_PI / (2.0f * 8) * (2.0f * n3 + 1) * k3);
								}
							}
						}
						// turn xyz to morton index
						size_t y_index = vvs::operation::XYZ2Index8(n1, n2, n3);

						this->RGB_[y_index].r_ = sum_r + 128, this->RGB_[y_index].g_ = sum_g + 128, this->RGB_[y_index].b_ = sum_b + 128;
					}
				}
			}
		}
	};

	// a compressed i-frame patch
	struct IFramePatch {
		// fitting patch octree
		std::string   octree_;
		pcl::PointXYZ center_;
		size_t        tree_height_;
		// fitting colors
		std::string colors_;
		// block number
		size_t block_number_;
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
		// block number
		size_t block_number_;
		// motion vector
		Eigen::Matrix4f motion_vector_;

		void clear() {
			this->octree_.clear();
			this->colors_.clear();
		}
	};

	struct TreeNode {
		uint8_t            subnodes_;
		float              sig_y_, sig_u_, sig_v_;
		std::vector<float> cof_y_, cof_u_, cof_v_;
		size_t             weight_;

		/***
		 * @description: default constructor
		 * @param {*}
		 * @return {*}
		 */
		TreeNode() = default;

		TreeNode(size_t __w) {
			this->weight_   = __w;
			this->subnodes_ = 0x00;
			this->sig_y_ = this->sig_u_ = this->sig_v_ = 0.0f;
		}
	};

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
}  // namespace operation
}  // namespace vvs

// References
// [1] Said Boussakta et. al. "Fast Algorithm for the 3-D DCT-II" IEEE TRANSACTIONS ON SIGNAL PROCESSING, 2004
// [2] Boon-Lock Yeo et. al. "Volume Rendering of DCT-Based Compressed 3D Scalar Data" IEEE TRANSACTIONS ON VISUALIZATION AND COMPUTER GRAPHICS, 1995

#endif