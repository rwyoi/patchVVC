/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 20:13:32
 * @LastEditTime: 2022-06-22 15:33:35
 * @LastEditors: ChenRP07
 * @Description:
 */
#ifndef _LIB_VIDEOCODER_H_
#define _LIB_VIDEOCODER_H_
#include "coder/io.h"
#include "coder/octree.h"
#include "coder/operation.hpp"
#include "coder/registration.h"
#include "coder/segment.h"
namespace vvs {
namespace coder {
	class Encoder {
	  private:
		std::vector<vvs::octree::GOF> point_clouds_;   // gof patches, i-th is the i-th patch including k frames
		const size_t                  kGroupOfFrames;  // constant number of gof
		const size_t                  kPatchNumber;    // constant number of patches
		const float                   kMinResolution;  // constant number of min resolution
		const size_t                  kThreads;
		const float                   kMSEThreshold;
		size_t                        frame_number_;  // count the encoded frames
		std::queue<size_t>            task_queue_;
		std::mutex                    task_mutex_, log_mutex_;

	  public:
		/***
		 * @description: constructor for encoder, assign values to all constant variable and init the point_clouds_
		 * @param {size_t} __GOF
		 * @param {size_t} __patches
		 * @param {float} __resolution
		 * @return {*}
		 */
		Encoder(const size_t, const size_t, const size_t, const float, const float = 2.0f);

		/***
		 * @description: add I-frame according the .ply file __file_name
		 * @param {string&} __file_name
		 * @return {*}
		 */
		void AddIFrame(const std::string&);

		/***
		 * @description: add p-frame according to the .ply file __file_name
		 * @param {string&} __file_name
		 * @return {*}
		 */
		void AddPFrame(const std::string&);

		/***
		 * @description: output __index-th frame to __file_name
		 * @param {string&} __file_name
		 * @param {size_t&} __index
		 * @return {*}
		 */
		void GetFrame(const std::string&, const size_t&);

		void FittingProc();
		void Fitting();
		void Output(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);
	};
}  // namespace coder
}  // namespace vvs

#endif