/***
 * @Author: ChenRP07
 * @Date: 2022-06-21 14:45:43
 * @LastEditTime: 2022-06-30 15:37:40
 * @LastEditors: ChenRP07
 * @Description: implement of point cloud IO
 */
#include "dependency/io.h"

using namespace vvs;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

/***
 * @description: load cololed point cloud from a ply format file
 * @param {string&} file_name
 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
 * @return {}
 */
void io::LoadColorPlyFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	try {
		// file_name must be *.ply format
		std::regex  name_type{"^.*\\.ply$"};
		std::smatch matches;
		if (!std::regex_search(file_name, matches, name_type))
			throw "Wrong format file name";
	}
	// wrong file_name format
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " open failed : " << error_message << "." << std::endl;
		std::exit(1);
	}

	// open file
	FILE* fp = nullptr;
	try {
		fp = fopen(file_name.c_str(), "r");
		if (fp == nullptr)
			throw strerror(errno);
	}
	// open failed
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " open failed : " << error_message << "." << std::endl;
		fclose(fp);
		std::exit(1);
	}

	// check file format : ascii or binary
	char file_line[1024];
	int  file_type = -1;
	fgets(file_line, 1024, fp);  // read line : "ply"
	fgets(file_line, 1024, fp);  // read line : "format ..."

	char* format;
	try {
		if ((format = strstr(file_line, "format ascii")) != nullptr)
			file_type = 0;
		else if ((format = strstr(file_line, "format binary")) != nullptr)
			file_type = 1;
		else
			throw "File broken";
	}
	// wrong format
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " load failed : " << error_message << "." << std::endl;
		fclose(fp);
		std::exit(1);
	}

	// check points number
	int point_cloud_size = -1;
	try {
		char* end_header;
		char* num_line;
		while (1) {
			fgets(file_line, 1024, fp);

			// header end
			if ((end_header = strstr(file_line, "end_header")) != nullptr) {
				if (point_cloud_size == -1)
					throw "No point cloud size";

				break;
			}

			// element vertex %d
			if ((num_line = strstr(file_line, "element vertex ")) != nullptr) {
				point_cloud_size = 0;
				for (int i = 15; i < strlen(file_line); i++)
					if (file_line[i] >= '0' && file_line[i] <= '9') {
						point_cloud_size *= 10;
						point_cloud_size += (file_line[i] - '0');
					}
					else {
						break;
					}
			}
		}
	}
	// wrong number
	catch (const char* error_message) {
		std::cerr << "File" << file_name << " load failed : " << error_message << "." << std::endl;
		fclose(fp);
		std::exit(1);
	}

	// read data
	point_cloud.resize(point_cloud_size);
	// ascii
	if (file_type == 0) {
		for (size_t i = 0; i < point_cloud_size; i++) {
			int r, g, b;
			fscanf(fp, "%f %f %f %d %d %d", &point_cloud[i].x, &point_cloud[i].y, &point_cloud[i].z, &r, &g, &b);
			point_cloud[i].r = r;
			point_cloud[i].g = g;
			point_cloud[i].b = b;
		}
	}
	// binary
	else if (file_type == 1) {
		for (size_t i = 0; i < point_cloud_size; i++) {
			fread(&point_cloud[i], sizeof(pcl::PointXYZRGB), 1, fp);
		}
	}
	fclose(fp);
}

/***
 * @description: save point_cloud to file_name, using binary_mode ?
 * @param {string&} file_name
 * @param {string&} point_cloud
 * @param {bool} binary_mode
 * @return {}
 */
void io::SaveColorPlyFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud, const bool binary_mode) {
	try {
		// file_name must be *.ply format
		std::regex  name_type{"^.*\\.ply$"};
		std::smatch matches;
		if (!std::regex_search(file_name, matches, name_type))
			throw "Wrong format file name";
	}
	// wrong file_name format
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " open failed : " << error_message << "." << std::endl;
		std::exit(1);
	}

	// open file
	FILE* fp = nullptr;

	try {
		fp = fopen(file_name.c_str(), "w");
		if (fp == nullptr)
			throw strerror(errno);
	}
	// open failed
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " open failed : " << error_message << "." << std::endl;
		fclose(fp);
		std::exit(1);
	}

	// output header
	fprintf(fp, "%s\n", "ply");
	if (binary_mode)
		fprintf(fp, "%s\n", "format binary 1.0");
	else
		fprintf(fp, "%s\n", "format ascii 1.0");

	fprintf(fp, "%s %lu\n", "element vertex", point_cloud.size());

	fprintf(fp, "%s\n", "property float x");
	fprintf(fp, "%s\n", "property float y");
	fprintf(fp, "%s\n", "property float z");
	fprintf(fp, "%s\n", "property uchar red");
	fprintf(fp, "%s\n", "property uchar green");
	fprintf(fp, "%s\n", "property uchar blue");
	fprintf(fp, "%s\n", "end_header");

	// output data
	if (binary_mode) {
		for (size_t i = 0; i < point_cloud.size(); i++) {
			fwrite(&point_cloud[i], sizeof(pcl::PointXYZRGB), 1, fp);
		}
	}
	else {
		for (size_t i = 0; i < point_cloud.size(); i++) {
			fprintf(fp, "%.3f %.3f %.3f %u %u %u\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, point_cloud[i].r, point_cloud[i].g, point_cloud[i].b);
		}
	}

	fclose(fp);
}

/***
 * @description: save point_cloud to file_name, set color to unique_color, using binary_mode ?
 * @param {string&} file_name
 * @param {pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
 * @param {unsigned int} unique_color
 * @param {bool} binary_mode
 * @return {*}
 */
void io::SaveUniqueColorPlyFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud, unsigned int unique_color, const bool binary_mode) {
	try {
		// file_name must be *.ply format
		std::regex  name_type{"^.*\\.ply$"};
		std::smatch matches;
		if (!std::regex_search(file_name, matches, name_type))
			throw "Wrong format file name";
	}
	// wrong file_name format
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " open failed : " << error_message << "." << std::endl;
		std::exit(1);
	}

	// open file
	FILE* fp = nullptr;

	try {
		fp = fopen(file_name.c_str(), "w");
		if (fp == nullptr)
			throw strerror(errno);
	}
	// open failed
	catch (const char* error_message) {
		std::cerr << "File " << file_name << " open failed : " << error_message << "." << std::endl;
		fclose(fp);
		std::exit(1);
	}

	// output header
	fprintf(fp, "%s\n", "ply");
	if (binary_mode)
		fprintf(fp, "%s\n", "format binary 1.0");
	else
		fprintf(fp, "%s\n", "format ascii 1.0");

	fprintf(fp, "%s %lu\n", "element vertex", point_cloud.size());

	fprintf(fp, "%s\n", "property float x");
	fprintf(fp, "%s\n", "property float y");
	fprintf(fp, "%s\n", "property float z");
	fprintf(fp, "%s\n", "property uchar red");
	fprintf(fp, "%s\n", "property uchar green");
	fprintf(fp, "%s\n", "property uchar blue");
	fprintf(fp, "%s\n", "end_header");

	unsigned char blue = unique_color & 0x0000ff;
	unique_color >>= 8;
	unsigned char green = unique_color & 0x0000ff;
	unique_color >>= 8;
	unsigned char red = unique_color & 0x0000ff;

	// output data
	if (binary_mode) {
		for (size_t i = 0; i < point_cloud.size(); i++) {
			pcl::PointXYZRGB temp_point(point_cloud[i]);
			temp_point.r = red, temp_point.g = green, temp_point.b = blue;

			fwrite(&temp_point, sizeof(pcl::PointXYZRGB), 1, fp);
		}
	}
	else {
		for (size_t i = 0; i < point_cloud.size(); i++) {
			fprintf(fp, "%.3f %.3f %.3f %u %u %u\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, red, green, blue);
		}
	}

	fclose(fp);
}

#pragma GCC diagnostic pop
