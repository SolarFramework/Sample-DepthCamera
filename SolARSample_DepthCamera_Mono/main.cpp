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

// ADD MODULES TRAITS HEADERS HERE
#include "SolARModuleOpencv_traits.h"
#include "SolARModuleRealSense_traits.h"
#include "SolARModulePCL_traits.h"
#include "SolARModuleOpengl_traits.h"
#include "SolARModuleTools_traits.h"

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
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::PCL;
using namespace SolAR::MODULES::REALSENSE;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::MODULES::OPENGL;

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
		auto pcFilter = xpcfComponentManager->create<SolARPCFilter>()->bindTo<pointCloud::IPCFilter>();
		auto pcFilterCentroid = xpcfComponentManager->create<SolARPCFilterCentroid>()->bindTo<pointCloud::IPCFilterCentroid>();
		auto icp = xpcfComponentManager->create<SolARICP>()->bindTo<solver::pose::I3DTransformFinderFrom3D3D>();
		auto icpNormals = xpcfComponentManager->create<SolARICPNormals>()->bindTo<solver::pose::I3DTransformFinderFrom3D3D>();
		auto transform3D = xpcfComponentManager->create<SolAR3DTransform>()->bindTo<geom::I3DTransform>();
		auto overlay2DCenter = xpcfComponentManager->create<SolAR2DOverlayOpencv>("center")->bindTo<display::I2DOverlay>();
		auto overlay2DPoints = xpcfComponentManager->create<SolAR2DOverlayOpencv>("points")->bindTo<display::I2DOverlay>();
		auto viewerRGB = xpcfComponentManager->create<SolARImageViewerOpencv>("color")->bindTo<display::IImageViewer>();
		auto viewerDepth = xpcfComponentManager->create<SolARImageViewerOpencv>("depth")->bindTo<display::IImageViewer>();
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
		pcLoader->load(pcLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue(),meshPointCloud);

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
                transform3D->transformInPlace(targetPointCloud, init_pose);
                Transform3Df pose_coarse, pose_fine;

                /// coarse pose estimate
                icp->estimate(sourcePointCloud, targetPointCloud, pose_coarse);
                /// update source cloud pose
                transform3D->transformInPlace(sourcePointCloud, pose_coarse);
                // refined pose estimate
                icpNormals->estimate(sourcePointCloud, targetPointCloud, pose_fine);
                transform3D->transformInPlace(targetPointCloud, (pose_fine*pose_coarse).inverse());
                transform3D->transformInPlace(ofilteredPointCloud, (pose_fine*pose_coarse).inverse());
			}

            // draw mesh overlay
            if (targetPointCloud != nullptr)
            {
				overlay2DPoints->drawCircles(camera->getWorldToPixels(targetPointCloud->getConstPointCloud()), imageRGB);
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
