/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 20:13:32
 * @LastEditTime: 2022-06-29 16:34:25
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
#include <sys/io.h>
#include <unistd.h>
#include <sys/stat.h>

namespace vvs {
namespace coder {
	class Encoder {
	  private:
		std::vector<vvs::octree::GOF>                    point_clouds_;  // gof patches, i-th is the i-th patch including k frames
		std::vector<vvs::type::IFramePatch>              i_frame_patches_;
		std::vector<std::vector<vvs::type::PFramePatch>> p_frame_patches_;
		const size_t                                     kGroupOfFrames;  // constant number of gof
		const size_t                                     kPatchNumber;    // constant number of patches
		const float                                      kMinResolution;  // constant number of min resolution
		const size_t                                     kThreads;
		const float                                      kMSEThreshold;
		size_t                                           frame_number_;  // count the encoded frames
		std::queue<size_t>                               task_queue_;
		std::mutex                                       task_mutex_, log_mutex_;

		void EncodingProc();

	  public:
		/***
		 * @description: constructor for encoder, assign values to all constant variable and init the point_clouds_
		 * @param {size_t} __GOF
		 * @param {size_t} __patches
		 * @param {size_t} __threads
		 * @param {float} __mse_ths
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

		void Encoding();
		void Output(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		void OutputIFrame(const std::string& __i_frame_name);
		void OutputPFrame(const std::string& __p_frame_name);
	};
}  // namespace coder
}  // namespace vvs

#endif