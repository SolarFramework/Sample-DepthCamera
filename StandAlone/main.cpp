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

#define USE_FREE
#include <iostream>
#include <string>
#include <vector>

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleRealSense_traits.h"
#include "SolARModulePCL_traits.h"
#include "SolARModuleOpengl_traits.h"
#include "SolARModuleTools_traits.h"

#ifndef USE_FREE
#include "SolARModuleNonFreeOpencv_traits.h"
#endif

#include "xpcf/xpcf.h"

#include "api/input/devices/IRGBDCamera.h"
#include "api/input/files/IPointCloudLoader.h"
#include "api/geom/I3DTransform.h"
#include "api/pointCloud/IPCFilterCentroid.h"
#include "api/pointCloud/IPCFilter.h"
#include "api/solver/pose/I3DTransformFinderFrom3D3D.h"
#include "api/image/IImageConvertor.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

#include "core/Log.h"

#define DEPTH_SCALE 4.f * 255.0f/65535.0f

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::PCL;
using namespace SolAR::MODULES::REALSENSE;
using namespace SolAR::MODULES::TOOLS;
#ifndef USE_FREE
using namespace SolAR::MODULES::NONFREEOPENCV;
#endif
using namespace SolAR::MODULES::OPENGL;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv){

//#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
//#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
#ifdef USE_FREE
    if(xpcfComponentManager->load("conf_DepthCamera.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_DepthCamera.xml")
        return -1;
    }
#else
    if(xpcfComponentManager->load("conf_Triangulation_nf.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Triangulation_nf.xml")
        return -1;
    }
#endif
    // declare and create components
    LOG_INFO("Start creating components");
    // component declaration and creation
    SRef<input::devices::IRGBDCamera> camera =xpcfComponentManager->create<SolARRGBDCamera>()->bindTo<input::devices::IRGBDCamera>();
	SRef<input::files::IPointCloudLoader> pcLoader =xpcfComponentManager->create<SolARPointCloudLoader>()->bindTo<input::files::IPointCloudLoader>();
	SRef<image::IImageConvertor> imageConvertor =xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
	SRef<pointCloud::IPCFilter> pcFilter = xpcfComponentManager->create<SolARPCFilter>()->bindTo<pointCloud::IPCFilter>();
	SRef<pointCloud::IPCFilterCentroid> pcFilterCentroid = xpcfComponentManager->create<SolARPCFilterCentroid>()->bindTo<pointCloud::IPCFilterCentroid>();
	SRef<solver::pose::I3DTransformFinderFrom3D3D> icp = xpcfComponentManager->create<SolARICP>()->bindTo<solver::pose::I3DTransformFinderFrom3D3D>();
	SRef<solver::pose::I3DTransformFinderFrom3D3D> icpNormals = xpcfComponentManager->create<SolARICPNormals>()->bindTo<solver::pose::I3DTransformFinderFrom3D3D>();
	SRef<geom::I3DTransform> transform3D = xpcfComponentManager->create<SolAR3DTransform>()->bindTo<geom::I3DTransform>();
	SRef<display::I2DOverlay> overlay2DCenter = xpcfComponentManager->create<SolAR2DOverlayOpencv>("center")->bindTo<display::I2DOverlay>();
	SRef<display::I2DOverlay> overlay2DPoints = xpcfComponentManager->create<SolAR2DOverlayOpencv>("points")->bindTo<display::I2DOverlay>();
	SRef<display::IImageViewer> viewerRGB =xpcfComponentManager->create<SolARImageViewerOpencv>("color")->bindTo<display::IImageViewer>();
	SRef<display::IImageViewer> viewerDepth =xpcfComponentManager->create<SolARImageViewerOpencv>("depth")->bindTo<display::IImageViewer>();
	SRef<display::I3DPointsViewer> viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    // declarations of data structures used to exange information between components
    SRef<Image> imageRGB;
    SRef<Image> imageDepth;
    SRef<Image> imageConvertedDepth = xpcf::utils::make_shared<Image>(Image::LAYOUT_GREY, Image::PER_CHANNEL, Image::DataType::TYPE_8U);
    SRef<PointCloud> meshPointCloud;
    SRef<PointCloud> pointCloud;
    SRef<PointCloud> downsampledPointCloud;
    SRef<PointCloud> filteredPointCloud;

    // pointclouds used for registration
    SRef<PointCloud> sourcePointCloud; // the measured pointcloud
    SRef<PointCloud> targetPointCloud; // the model reference pointclouud
		

    // load mesh
    pcLoader->load("frac_star.pcd", meshPointCloud);
    std::cout<<"NB points "<< meshPointCloud->getPointCloud().size() << std::endl;

    // start depth camera
    if(camera->startRGBD() != FrameworkReturnCode::_SUCCESS) {
        LOG_ERROR("Can't start camera stream. Check if camera is connected on a USB3 port.")
        return EXIT_FAILURE;
    }

    char lastKey = ' ';
    bool registered = false;

    // Display the matches and the 3D point cloud
    while (true){
        camera->getNextRGBDFrame(imageRGB, imageDepth);
        camera->getPointCloud(pointCloud);

        overlay2DCenter->drawCircle(Point2Df(640,360), imageRGB);

        // downsample on new grid
        pcFilter->filter(pointCloud,downsampledPointCloud);

        if( lastKey =='r' )
        {
            // filter given centroid point
            SRef<Point3Df> centroid( new Point3Df( camera->getPixelToWorld( { 640, 360 } ) ) ); // middle of the screen

            std::cout << "centroid selection coord @ " << centroid->getX() << "," << centroid->getY() << "," << centroid->getZ() << std::endl;

            pcFilterCentroid->filter(downsampledPointCloud, centroid, filteredPointCloud);

            // deep copy pointclouds to have a work items
            sourcePointCloud = xpcf::utils::make_shared<PointCloud>( *filteredPointCloud );
            targetPointCloud = xpcf::utils::make_shared<PointCloud>( *meshPointCloud );

            // translate target mesh to be behind measured pointcloud
            Transform3Df init_pose = Transform3Df::Identity();
            init_pose.translate( Vector3f{ centroid->x(), centroid->y(), centroid->z() + 0.05f } );
            transform3D->transformInPlace(targetPointCloud,init_pose);

            Transform3Df pose_coarse, pose_fine;

            // coarse pose estimate
            icp->estimate(sourcePointCloud, targetPointCloud, pose_coarse);
            std::cout << "coarse pose : \n" << pose_coarse.matrix() << std::endl;

            // update source cloud pose
            transform3D->transformInPlace(sourcePointCloud,pose_coarse);

            // refined pose estimate
            icpNormals->estimate(sourcePointCloud, targetPointCloud, pose_fine);
            std::cout << "refined pose : \n" << (pose_fine*pose_coarse).matrix() << std::endl;
            transform3D->transformInPlace(targetPointCloud,(pose_fine*pose_coarse).inverse());
        }
        else
        {
            // draw mesh overlay
            if ( targetPointCloud != nullptr )
            {
                overlay2DPoints->drawCircles(camera->getWorldToPixels(targetPointCloud->getConstPointCloud()), imageRGB);
                //std::cout<<"Display "<< targetPointCloud->getConstPointCloud().size() << " points" << std::endl;
            }
        }

        imageConvertor->convert(imageDepth, imageConvertedDepth, Image::LAYOUT_GREY, DEPTH_SCALE);
        if ( viewerRGB->displayKey(imageRGB,lastKey) == FrameworkReturnCode::_STOP  ||
             viewerDepth->display(imageConvertedDepth) == FrameworkReturnCode::_STOP ||
             viewer3DPoints->display(downsampledPointCloud, Transform3Df::Identity(), {}, {}, filteredPointCloud) == FrameworkReturnCode::_STOP)
        {
           LOG_INFO("End of Depth Camera sample");
           break;
        }
    }
    return 0;
}


