/*
 * WmObjectRecognition.h
 *
 *  Created on: 26/10/2015
 *      Author: paco
 */

#ifndef WMOBJECTRECOGNITION_H_
#define WMOBJECTRECOGNITION_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/recognition/obj_rec_ransac.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <wm_objects/Objects.h>

namespace wm_object {

class WmObjectRecognition {
public:
	WmObjectRecognition();
	virtual ~WmObjectRecognition();

	void loadModelFromFile(const std::string& filename, const std::string& objectname);

private:
	double pairWidth_;
	float voxelSize_;

	wm_objects::Objects objects_filenames_;

	pcl::recognition::ObjRecRANSAC recognition_;

	void normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud_o_Ptr, pcl::PointCloud<pcl::Normal>::Ptr normalCloud_o_Ptr, double radius_d);
	bool recognize(pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudPtr,
			pcl::PointCloud<pcl::Normal>::Ptr sceneNormalsPtr,
			std::list<pcl::recognition::ObjRecRANSAC::Output>& output);
};

} /* namespace wm_object */

#endif /* WMOBJECTRECOGNITION_H_ */
