/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Improved by drag and bot GmbH (www.dragandbot.com), 2019
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef PYLON_CAMERA_PYLON_CAMERA_NODE_H
#define PYLON_CAMERA_PYLON_CAMERA_NODE_H

#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_camera.h>

#include <camera_control_msgs/SetBinning.h>
#include <camera_control_msgs/SetBrightness.h>
#include <camera_control_msgs/SetExposure.h>
#include <camera_control_msgs/SetGain.h>
#include <camera_control_msgs/SetGamma.h>
#include <camera_control_msgs/SetROI.h>
#include <camera_control_msgs/SetSleeping.h>
#include <camera_control_msgs/SetIntegerValue.h>
#include <camera_control_msgs/SetFloatValue.h>
#include <camera_control_msgs/SetStringValue.h>
#include <camera_control_msgs/currentParams.h>
#include <camera_control_msgs/SetWhiteBalance.h>
#include <camera_control_msgs/GetIntegerValue.h>
#include <camera_control_msgs/GetFloatValue.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <camera_control_msgs/GrabImagesAction.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dnb_msgs/ComponentStatus.h>


namespace pylon_camera
{

typedef actionlib::SimpleActionServer<camera_control_msgs::GrabImagesAction> GrabImagesAS;

/**
 * The ROS-node of the pylon_camera interface
 */
class PylonCameraNode
{
public:

    PylonCameraNode();
    virtual ~PylonCameraNode();

    /**
     * initialize the camera and the ros node.
     * calls ros::shutdown if an error occurs.
     */
    void init();

    /**
     * spin the node
     */
    virtual void spin();

    /**
     * Getter for the frame rate set by the launch script or from the ros parameter
     * server
     * @return the desired frame rate.
     */
    const double& frameRate() const;

    /**
     * Getter for the tf frame.
     * @return the camera frame.
     */
    const std::string& cameraFrame() const;

protected:
    /**
     * Creates the camera instance and starts the services and action servers.
     * @return false if an error occurred
     */
    bool initAndRegister();
    bool startGrabbing();
    void setupRectification();
    virtual bool grabImage();
    virtual void setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg);

    /**
     * Update area of interest in the camera image
     * @param target_roi the target roi
     * @param reached_roi the roi that could be set
     * @return true if the targeted roi could be reached
     */
    bool setROI(const sensor_msgs::RegionOfInterest target_roi,
                sensor_msgs::RegionOfInterest& reached_roi);
    bool setExposure(const float& target_exposure, float& reached_exposure);
    bool setBrightness(const int& target_brightness,
                       int& reached_brightness,
                       const bool& exposure_auto,
                       const bool& gain_auto);
    bool setGain(const float& target_gain, float& reached_gain);
    bool setGamma(const float& target_gamma, float& reached_gamma);
    bool isSleeping();
    void setupSamplingIndices(std::vector<std::size_t>& indices,
                              std::size_t rows,
                              std::size_t cols,
                              int downsampling_factor);
    void genSamplingIndicesRec(std::vector<std::size_t>& indices,
                               const std::size_t& min_window_height,
                               const cv::Point2i& start,
                               const cv::Point2i& end);

    float calcCurrentBrightness();

    /**
     * Callback for the grab images action
     * @param goal the goal
     */
    void grabImagesRawActionExecuteCB(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);

    /**
     * Callback for the grab rectified images action
     * @param goal the goal
     */
    void grabImagesRectActionExecuteCB(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);
    /**
     * This function can also be called from the derived PylonCameraOpenCV-Class
     */
    camera_control_msgs::GrabImagesResult grabImagesRaw(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal,
                    GrabImagesAS* action_server);

    void initCalibrationMatrices(sensor_msgs::CameraInfo& info,
                                 const cv::Mat& D,
                                 const cv::Mat& K);




    /**
     * Waits till the pylon_camera_ isReady() observing a given timeout
     * @return true when the camera's state toggles to 'isReady()'
     */
    bool waitForCamera(const ros::Duration& timeout) const;






    /**
     * Method to reset the camera device
     * @return error message if an error occurred or done message otherwise.
     */
    std::string triggerDeviceReset();



    /**
     * Method to starting camera aqcuisition
     * @return error message if an error occurred or done message otherwise.
     */
    std::string grabbingStarting();



    /**
     * Method to stopping camera aqcuisition
     * @return error message if an error occurred or done message otherwise.
     */
    std::string grabbingStopping();

    /**
     * Method to collect and publish the current camera parameters
     */
    void currentParamPub();



    /**
     * Method to set the camera image encoding
     * @param target_ros_endcoding: string describing the encoding (mono8, mono16, bgr8, rgb8, bayer_bggr8, bayer_gbrg8, bayer_rggb8, bayer_grbg8, bayer_rggb16, bayer_bggr16, bayer_gbrg16, Bayer_grbg16).
     * @return false if a communication error occurred or true otherwise.
     */
    std::string setImageEncoding(const std::string& target_ros_encoding);







    ros::NodeHandle nh_;
    PylonCameraParameter pylon_camera_parameter_set_;

    ros::ServiceServer reset_device_srv_;



    // DNB component status publisher
    ros::Publisher componentStatusPublisher;
    dnb_msgs::ComponentStatus cm_status;

    // current params publisher
    ros::Publisher currentParamsPublisher;
    camera_control_msgs::currentParams params;




    PylonCamera* pylon_camera_;

    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher img_raw_pub_;
    ros::Publisher image_numbered_pub_;

    image_transport::Publisher* img_rect_pub_;
    image_geometry::PinholeCameraModel* pinhole_model_;

    GrabImagesAS grab_imgs_raw_as_;
    GrabImagesAS* grab_imgs_rect_as_;

    sensor_msgs::Image img_raw_msg_;
    image_numbered_msgs::ImageNumbered raw_img_numbered_;
    cv_bridge::CvImage* cv_bridge_img_rect_;

    camera_info_manager::CameraInfoManager* camera_info_manager_;

    std::vector<std::size_t> sampling_indices_;
    std::array<float, 256> brightness_exp_lut_;

    bool is_sleeping_;
    boost::recursive_mutex grab_mutex_;

    /// diagnostics:
    diagnostic_updater::Updater diagnostics_updater_;
    void diagnostics_timer_callback_(const ros::TimerEvent&);
    ros::Timer diagnostics_trigger_;
    void create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void configMasterCam();
    void create_camera_info_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_NODE_H
