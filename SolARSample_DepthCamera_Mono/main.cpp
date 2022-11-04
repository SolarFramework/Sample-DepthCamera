/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <string>
#include <vector>
#include <boost/log/core.hpp>


// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/IRGBDCamera.h"
#include "api/input/files/IPointCloudLoader.h"
#include "api/geom/I3DTransform.h"
#include "api/pointCloud/IPCFilterCentroid.h"
#include "api/pointCloud/IPCFilter.h"
#include "api/solver/pose/I3DTransformFinderFrom3D3D.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv) {
	#pragma region initialization
	#if NDEBUG
		boost::log::core::get()->set_logging_enabled(false);
	#endif
	
	LOG_ADD_LOG_TO_CONSOLE();
	try {
		/* instantiate component manager*/
		/* this is needed in dynamic mode */
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if (xpcfComponentManager->load("SolARSample_DepthCamera_Mono_conf.xml") != org::bcom::xpcf::_SUCCESS)
		{
            LOG_ERROR("Failed to load the configuration file SolARSample_DepthCamera_Mono_conf.xml")
				return -1;
		}
	#pragma endregion

		#pragma region declaration components 
		LOG_INFO("Start creating components");
		// component declaration and creation
		auto camera = xpcfComponentManager->resolve<input::devices::IRGBDCamera>();
		auto pcLoader = xpcfComponentManager->resolve<input::files::IPointCloudLoader>();
        auto pcFilter = xpcfComponentManager->resolve<pointCloud::IPCFilter>();
        auto pcFilterCentroid = xpcfComponentManager->resolve<pointCloud::IPCFilterCentroid>();
        auto icp = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom3D3D>();
        auto icpNormals = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom3D3D>();
        auto transform3D = xpcfComponentManager->resolve<geom::I3DTransform>();
        auto overlay2DCenter = xpcfComponentManager->resolve<display::I2DOverlay>("center");
        auto overlay2DPoints = xpcfComponentManager->resolve<display::I2DOverlay>("points");
        auto viewerRGB = xpcfComponentManager->resolve<display::IImageViewer>("color");
        auto viewerDepth = xpcfComponentManager->resolve<display::IImageViewer>("depth");
        auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		LOG_INFO("Components created");
		#pragma endregion

		#pragma region declarations
		SRef<Image>				imageRGB;
		SRef<Image>				imageDepth;
		SRef<PointCloud>		meshPointCloud;
		SRef<PointCloud>		pointCloud;
		SRef<PointCloud>		downsampledPointCloud;
		SRef<PointCloud>		filteredPointCloud;

		// pointclouds used for registration
		/// the measured pointcloud
		SRef<PointCloud>		sourcePointCloud; 
		/// the model reference pointcloud
		SRef<PointCloud>		targetPointCloud; 
		///filtered reference point cloud
		SRef<PointCloud>		ofilteredPointCloud; 
		#pragma endregion

		// load mesh
        pcLoader->load(meshPointCloud);

        // start depth camera
        if (camera->start() != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Can't start camera stream. Check if camera is connected on a USB3 port.")
                return EXIT_FAILURE;
        }

        char lastKey = ' ';
		//enable to loop on frames
        bool debug_loop = false;
		//enable to watch filtered cloud point and refine pose cloud point in the 3D view
		bool debug_3d = false; 

        while (true) {
            camera->getNextRGBDFrame(imageRGB, imageDepth);
            camera->getPointCloud(pointCloud);
            overlay2DCenter->drawCircle(Point2Df(imageRGB.get()->getWidth() / 2, imageRGB.get()->getHeight() / 2), imageRGB);

            // downsample on new grid
            pcFilter->filter(pointCloud, downsampledPointCloud);

			if (lastKey == 'l') { debug_loop = !debug_loop; LOG_INFO("Debug loop {}", debug_loop); }
			if (lastKey == 'd') { debug_3d = !debug_3d; LOG_INFO("Debug 3D {}", debug_3d); }
			
            if (lastKey == 'r' || debug_loop)
            {
                // filter given centroid point
				SRef<Point3Df> centroid(new Point3Df(camera->getPixelToWorld({ static_cast<int32_t>(camera->getDepthResolution().width) / 2, static_cast<int32_t>(camera->getDepthResolution().height) / 2 }))); // middle of the screen
				LOG_DEBUG("centroid selection coord @ {},{},{}", centroid->getX(), centroid->getY(), centroid->getZ());
				if (centroid->getZ() < camera->getDepthMinDistance()) LOG_ERROR("Centroid is too close of depth sensor {} , min distance is {}", centroid->getZ(), camera->getDepthMinDistance());
				
                pcFilterCentroid->filter(downsampledPointCloud, centroid, filteredPointCloud);

                /// deep copy pointclouds to have a work items
                sourcePointCloud = xpcf::utils::make_shared<PointCloud>(*filteredPointCloud);
				ofilteredPointCloud = xpcf::utils::make_shared<PointCloud>(*filteredPointCloud);
                targetPointCloud = xpcf::utils::make_shared<PointCloud>(*meshPointCloud);

				/// translate target mesh to be behind measured pointcloud
                Transform3Df init_pose = Transform3Df::Identity();
				///move forward to help ICP to converge in a non minimum local
                init_pose.translate(Vector3f{ centroid->x(), centroid->y(), centroid->z() + 0.05f }); 
                transform3D->transformInPlace(init_pose, targetPointCloud);
                Transform3Df pose_coarse, pose_fine;

                /// coarse pose estimate
                icp->estimate(sourcePointCloud, targetPointCloud, pose_coarse);
                /// update source cloud pose
                transform3D->transformInPlace(pose_coarse, sourcePointCloud);
                // refined pose estimate
                icpNormals->estimate(sourcePointCloud, targetPointCloud, pose_fine);
                transform3D->transformInPlace((pose_fine*pose_coarse).inverse(), targetPointCloud);
                transform3D->transformInPlace((pose_fine*pose_coarse).inverse(), ofilteredPointCloud);
			}

            // draw mesh overlay
            if (targetPointCloud != nullptr)
            {
				std::vector<SRef<CloudPoint>> cloudPoints;
				targetPointCloud->getAllPoints(cloudPoints);
				overlay2DPoints->drawCircles(camera->getWorldToPixels(cloudPoints), imageRGB);
				if (debug_3d) {
					viewer3DPoints->display(ofilteredPointCloud, Transform3Df::Identity(), {}, {}, filteredPointCloud);
				}
            }

            if (viewerRGB->displayKey(imageRGB, lastKey) == FrameworkReturnCode::_STOP ||
                viewerDepth->display(imageDepth) == FrameworkReturnCode::_STOP ||
                (!debug_3d && viewer3DPoints->display(downsampledPointCloud, Transform3Df::Identity(), {}, {}, filteredPointCloud) == FrameworkReturnCode::_STOP))
            {
                LOG_INFO("End of Depth Camera sample");
                break;
            }
        }
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR("The following exception has been catch : {}", e.what());
		return -1;
	}
    return 0;
}
