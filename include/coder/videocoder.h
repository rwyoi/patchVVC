/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 20:13:32
 * @LastEditTime: 2022-07-05 16:47:44
 * @LastEditors: ChenRP07
 * @Description: Header of Volumetric Video Encoder
 */
#ifndef _LIB_VIDEOCODER_H_
#define _LIB_VIDEOCODER_H_
#include "dependency/io.h"
#include "dependency/octree.h"
#include "dependency/operation.hpp"
#include "dependency/registration.h"
#include "dependency/segment.h"
#include "dependency/type.hpp"
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

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

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> last_patches_;
		std::vector<Eigen::Matrix4f>                   last_motions_;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> test_;

		/***
		 * @description: task function for multi-threads encoding
		 * @param {*}
		 * @return {*}
		 */
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

		/***
		 * @description: multi-threads encoding
		 * @param {*}
		 * @return {*}
		 */
		void Encoding();

		void Output(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

		/***
		 * @description: write i-frame to __i_frame_name
		 * @param {string&} __i_frame_name
		 * @return {*}
		 */
		void OutputIFrame(const std::string& __i_frame_name);

		/***
		 * @description: write p-frame to __p_frame_name
		 * @param {string&} __p_frame_name
		 * @return {*}
		 */
		void OutputPFrame(const std::string& __p_frame_name);
	};

	class Decoder {
	  private:
		std::vector<pcl::PointCloud<pcl::PointXYZ>>   fitting_patches_;
		std::vector<std::vector<vvs::type::ColorRGB>> fitting_colors_;
		std::vector<vvs::type::IFramePatch>           I_Frame_Patches_;
		std::vector<vvs::type::PFramePatch>           P_Frame_Patches_;

		std::vector<pcl::PointCloud<pcl::PointXYZ>>   single_patches_;
		std::vector<std::vector<vvs::type::ColorRGB>> p_colors_;

		std::queue<size_t> task_pool_;
		std::mutex         task_mutex_;

		size_t kGroupOfFrame;
		size_t kPatchNumber;
		float  kMinResolution;

		size_t frame_number_;

		const size_t kThreads;

		void GetColorProc(size_t index, int __frame);
		void GetIFrameProc();
		void GetPFrameProc();

	  public:
		Decoder(const size_t __ths);
		void AddIFrame(const std::string& __i_frame_name);
		void AddPFrame(const std::string& __p_frame_name);
		void GetIFrame(pcl::PointCloud<pcl::PointXYZRGB>& __i_frame);
		void GetPFrame(pcl::PointCloud<pcl::PointXYZRGB>& __p_frame);
	};

}  // namespace coder
}  // namespace vvs

#endif