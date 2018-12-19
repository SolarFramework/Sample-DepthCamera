cmake_minimum_required(VERSION 3.7.2)

##################################################
project("SolARDepthCameraSample")
set (SOURCES main.cpp)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()

# define the list of files to copy to build directory
set(FILES_TO_COPY
	$ENV{BCOMDEVROOT}/thirdParties/opencv/${OPENCV_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_world${OPENCVVERSIONSUFFIX}.${LIBEXTENSION}*

    $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_context.${LIBEXTENSION}*
    $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_filesystem.${LIBEXTENSION}*
    $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_system.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_timer.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_log.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_chrono.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_thread.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_date_time.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_fiber.${LIBEXTENSION}*
	
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_common_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_features_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_filters_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_io_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_io_ply_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_kdtree_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_keypoints_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_ml_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_octree_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_recognition_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_registration_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_sample_consensus_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_search_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_segmentation_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_stereo_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_surface_${BUILDCONFIG}.${LIBEXTENSION}*
	$ENV{BCOMDEVROOT}/thirdParties/pcl/${PCL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}pcl_tracking_${BUILDCONFIG}.${LIBEXTENSION}*

	$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/${SOLARFRAMEWORK_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARFramework.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/xpcf/${XPCF_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}xpcf.${LIBEXTENSION}	

	$ENV{BCOMDEVROOT}/thirdParties/freeglut/${FREEGLUT_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}${LIBGLUT}glut.${LIBEXTENSION}*
	
	$ENV{BCOMDEVROOT}/thirdParties/realsense2/${REALSENSE_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}$???.${LIBEXTENSION}*

	${CMAKE_CURRENT_SOURCE_DIR}/conf_DepthCamera.xml
	)
# define targets (library, install and uninstall)
defineTargets("executable" "${FILES_TO_COPY}")

