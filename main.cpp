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
#include "api/pointCloud/IPCFilterCentroid.h"
#include "api/pointCloud/IPCFilter.h"
#include "api/solver/pose/I3DTransformFinderFrom3D3D.h"
#include "api/image/IImageConvertor.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

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

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

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
    SRef<input::devices::IRGBDCamera> camera =xpcfComponentManager->create<RGBDCamera>()->bindTo<input::devices::IRGBDCamera>();
    SRef<input::files::IPointCloudLoader> pcLoader =xpcfComponentManager->create<PointCloudLoader>()->bindTo<input::files::IPointCloudLoader>();
    SRef<image::IImageConvertor> imageConvertor =xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    SRef<pointCloud::IPCFilter> pcFilter = xpcfComponentManager->create<PCFilter>()->bindTo<pointCloud::IPCFilter>();
    SRef<pointCloud::IPCFilterCentroid> pcFilterCentroid = xpcfComponentManager->create<PCFilterCentroid>()->bindTo<pointCloud::IPCFilterCentroid>();
    SRef<solver::pose::I3DTransformFinderFrom3D3D> icp = xpcfComponentManager->create<ICP>()->bindTo<solver::pose::I3DTransformFinderFrom3D3D>();
    SRef<solver::pose::I3DTransformFinderFrom3D3D> icpNormals = xpcfComponentManager->create<ICPNormals>()->bindTo<solver::pose::I3DTransformFinderFrom3D3D>();
    SRef<display::I2DOverlay> overlay2D = xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();
    SRef<display::IImageViewer> viewerRGB =xpcfComponentManager->create<SolARImageViewerOpencv>("color")->bindTo<display::IImageViewer>();
    SRef<display::IImageViewer> viewerDepth =xpcfComponentManager->create<SolARImageViewerOpencv>("depth")->bindTo<display::IImageViewer>();
    SRef<display::I3DPointsViewer> viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    // declarations of data structures used to exange information between components
    SRef<Image> imageRGB;
    SRef<Image> imageDepth;
    SRef<Image> imageConvertedDepth = xpcf::utils::make_shared<Image>(Image::LAYOUT_GREY, Image::PER_CHANNEL, Image::DataType::TYPE_8U);
    SRef<PointCloud> pointCloud;
    SRef<PointCloud> downsampledPointCloud;
    SRef<PointCloud> filteredPointCloud;

    if(camera->startRGBD() != FrameworkReturnCode::_SUCCESS) {
        LOG_ERROR("Can't start camera stream. Check if camera is connected on a USB3 port.")
        return EXIT_FAILURE;
    }


    // Display the matches and the 3D point cloud
    while (true){
        camera->getNextRGBDFrame(imageRGB, imageDepth);
        camera->getPointCloud(pointCloud);

        // downsample on 5cm grid
        pcFilter->filter(pointCloud,downsampledPointCloud);

        // filter given centroid point
        SRef<Point3Df> centroid( new Point3Df( camera->getPixelToWorld( { 640, 360 } ) ) ); // middle of the screen
        pcFilterCentroid->filter(downsampledPointCloud, centroid, filteredPointCloud);

        imageConvertor->convert(imageDepth, imageConvertedDepth, Image::LAYOUT_GREY, DEPTH_SCALE);
        if ( viewerRGB->display(imageRGB) == FrameworkReturnCode::_STOP  ||
             viewerDepth->display(imageConvertedDepth) == FrameworkReturnCode::_STOP ||
             viewer3DPoints->display(downsampledPointCloud, Transform3Df::Identity(), {}, {}, filteredPointCloud) == FrameworkReturnCode::_STOP)
        {
           LOG_INFO("End of Depth Camera sample");
           break;
        }
    }
    return 0;
}



