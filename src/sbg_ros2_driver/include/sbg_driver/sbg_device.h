#ifndef SBG_ROS_SBG_DEVICE_H
#define SBG_ROS_SBG_DEVICE_H

// Standard headers
#include <iostream>
#include <map>
#include <string>

// ROS headers
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Project headers
#include <config_applier.h>
#include <config_store.h>
#include <message_publisher.h>

namespace sbg
{
/*!
 * Class to handle a connected SBG device.
 */
class SbgDevice
{
private:

  //---------------------------------------------------------------------//
  //- Static members definition                                         -//
  //---------------------------------------------------------------------//

  static std::map<SbgEComMagCalibQuality, std::string>    g_mag_calib_quality_;
  static std::map<SbgEComMagCalibConfidence, std::string> g_mag_calib_confidence_;
  static std::map<SbgEComMagCalibMode, std::string>       g_mag_calib_mode_;
  static std::map<SbgEComMagCalibBandwidth, std::string>  g_mag_calib_bandwidth;

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  SbgEComHandle           m_com_handle_;
  SbgInterface            m_sbg_interface_;
  rclcpp::Node&        	  m_ref_node_;
  MessagePublisher        m_message_publisher_;
  ConfigStore             m_config_store_;

  uint32_t                m_rate_frequency_;

  bool                    m_mag_calibration_ongoing_;
  bool                    m_mag_calibration_done_;
  SbgEComMagCalibResults  m_magCalibResults;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr    m_calib_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr      m_calib_save_service_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   *  Callback definition called each time a new log is received.
   * 
   *  \param[in]  pHandle         Valid handle on the sbgECom instance that has called this callback.
   *  \param[in]  msg_class       Class of the message we have received
   *  \param[in]  msg             Message ID of the log received.
   *  \param[in]  p_log_data      Contains the received log data as an union.
   *  \param[in]  p_user_arg      Optional user supplied argument.
   *  \return                     SBG_NO_ERROR if the received log has been used successfully.
   */
  static SbgErrorCode onLogReceivedCallback(SbgEComHandle* p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData* p_log_data, void* p_user_arg);

  /*!
   * Function to handle the received log.
   * 
   * \param[in]  msg_class        Class of the message we have received
   * \param[in]  msg              Message ID of the log received.
   * \param[in]  ref_sbg_data     Contains the received log data as an union.
   */
  void onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData& ref_sbg_data);

  /*!
   * Load the parameters.
   */
  void loadParameters(void);

  /*!
   * Create the connection to the SBG device.
   * 
   * \throw                       Unable to connect to the SBG device.
   */
  void connect(void);

  /*!
   * Read the device informations.
   *
   * \throw                       Unable to read the device information.
   */
  void readDeviceInfo(void);

  /*!
   * Get the SBG version as a string.
   * 
   * \param[in] sbg_version_enc   SBG version encoded.
   * \return                      String version decoded.
   */
  std::string getVersionAsString(uint32 sbg_version_enc) const;

  /*!
   * Initialize the publishers according to the configuration.
   */
  void initPublishers(void);

  /*!
   * Configure the connected SBG device.
   * This function will configure the device if the config file allows it.
   * It will log warning for unavailable parameters for the connected device.
   * 
   * \throw                       Unable to configure the connected device.
   */
  void configure(void);

  /*!
   * Process the magnetometer calibration.
   * 
   * \param[in] ref_ros_request   ROS service request.
   * \param[in] ref_ros_response  ROS service response.
   * \return                      Return true if the calibration process has been succesfull.
   */
  bool processMagCalibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> ref_ros_request, std::shared_ptr<std_srvs::srv::Trigger::Response> ref_ros_response);

  /*!
   * Save the magnetometer calibration.
   * 
   * \param[in] ref_ros_request   ROS service request.
   * \param[in] ref_ros_response  ROS service response.
   * \return                      Return true if the calibration has been saved.
   */
  bool saveMagCalibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> ref_ros_request, std::shared_ptr<std_srvs::srv::Trigger::Response> ref_ros_response);

  /*!
   * Start the magnetometer calibration process.
   * 
   * \return                      True if the calibration process has started successfully.
   */
  bool startMagCalibration(void);

  /*!
   * End the magnetometer calibration process.
   * 
   * \return                      True if the calibration process has ended successfully.
   */
  bool endMagCalibration(void);

  /*!
   * Upload the magnetometers calibration results to the device.
   * 
   * \return                      True if the magnetometers calibration has been successfully uploaded to the device.
   */
  bool uploadMagCalibrationToDevice(void);

  /*!
   * Display magnetometers calibration status result.
   */
  void displayMagCalibrationStatusResult(void) const;

  /*!
   * Export magnetometers calibration results.
   */
  void exportMagCalibrationResults(void) const;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   * 
   * \param[in] ref_node_handle   ROS Node.
   */
  SbgDevice(rclcpp::Node& ref_node_handle);

  /*!
   * Default destructor.
   */
  ~SbgDevice(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Get the frequency to update the main rate loop for device handling.
   * 
   * \return                      Device frequency to read the logs (in Hz).
   */
  uint32_t getUpdateFrequency(void) const;

  //---------------------------------------------------------------------//
  //- Public  methods                                                   -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the SBG device for receiving data.
   * 
   * \throw                       Unable to initialize the SBG device.
   */
  void initDeviceForReceivingData(void);

  /*!
   * Initialize the device for magnetometers calibration.
   */
  void initDeviceForMagCalibration(void);

  /*!
   * Periodic handle of the connected SBG device.
   */
  void periodicHandle(void);
};
}

#endif // SBG_ROS_SBG_DEVICE_H
