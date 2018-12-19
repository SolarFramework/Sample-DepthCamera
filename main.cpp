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
#include "SolARModuleOpengl_traits.h"
#include "SolARModuleTools_traits.h"

#ifndef USE_FREE
#include "SolARModuleNonFreeOpencv_traits.h"
#endif

#include "xpcf/xpcf.h"

#include "api/input/devices/IRGBDCamera.h"
#include "api/image/IImageConvertor.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

#define DEPTH_SCALE 4.f * 255.0f/65535.0f

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
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
    SRef<image::IImageConvertor> imageConvertor =xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    SRef<display::IImageViewer> viewerRGB =xpcfComponentManager->create<SolARImageViewerOpencv>("color")->bindTo<display::IImageViewer>();
    SRef<display::IImageViewer> viewerDepth =xpcfComponentManager->create<SolARImageViewerOpencv>("depth")->bindTo<display::IImageViewer>();
    //SRef<display::I3DPointsViewer> viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    // declarations of data structures used to exange information between components
    SRef<Image> imageRGB;
    SRef<Image> imageDepth;
    SRef<Image> imageConvertedDepth = xpcf::utils::make_shared<Image>(Image::LAYOUT_GREY, Image::PER_CHANNEL, Image::DataType::TYPE_8U);

    camera->startRGBD();

    // Display the matches and the 3D point cloud
    while (true){
        camera->getNextRGBDFrame(imageRGB, imageDepth);
        imageConvertor->convert(imageDepth, imageConvertedDepth, Image::LAYOUT_GREY, DEPTH_SCALE);
        if ( viewerRGB->display(imageRGB) == FrameworkReturnCode::_STOP  ||
             viewerDepth->display(imageConvertedDepth) == FrameworkReturnCode::_STOP)
        {
           LOG_INFO("End of Depth Camera sample");
           break;
        }
    }
    return 0;
}



