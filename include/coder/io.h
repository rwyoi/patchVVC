/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 14:44:11
 * @LastEditTime: 2022-06-21 15:17:54
 * @LastEditors: ChenRP07
 * @Description:
 */

#ifndef _LIB_IO_H_
#define _LIB_IO_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <regex>
#include <stdio.h>
#include <string>

namespace vvs {
namespace io {
	/***
	 * @description: load cololed point cloud from a ply format file
	 * @param {string&} file_name
	 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
	 * @return {}
	 */
	extern void LoadColorPlyFile(const std::string& __file_name, pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud);

	/***
	 * @description: save point_cloud to file_name, using binary_mode ?
	 * @param {string&} file_name
	 * @param {string&} point_cloud
	 * @param {bool} binary_mode
	 * @return {}
	 */
	extern void SaveColorPlyFile(const std::string& __file_name, const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const bool __binary_mode = false);

	/***
	 * @description: save point_cloud to file_name, set color to unique_color, using binary_mode ?
	 * @param {string&} file_name
	 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
	 * @param {unsigned int} unique_color
	 * @param {bool} binary_mode
	 * @return {*}
	 */
	extern void SaveUniqueColorPlyFile(const std::string& __file_name, const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, unsigned int __unique_color = 0x000000,
	                                   const bool __binary_mode = false);
}  // namespace io
}  // namespace vvs
#endif