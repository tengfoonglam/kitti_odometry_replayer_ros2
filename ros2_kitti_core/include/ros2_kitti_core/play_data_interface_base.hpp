#ifndef ROS2_KITTI_CORE__PLAY_DATA_INTERFACE_BASE_HPP_
#define ROS2_KITTI_CORE__PLAY_DATA_INTERFACE_BASE_HPP_

#include <cstdint>
#include <string>

namespace r2k_core
{

/**
 * @brief Interface in which DataReplayer triggers the preparation and playing of different data
 * sources
 *
 */
class PlayDataInterfaceBase
{
public:
  /**
   * @brief Construct a new Play Data Interface Base object
   *
   * @param name Name of interface
   */
  explicit PlayDataInterfaceBase(const std::string & name) : name_(name) {}

  /**
   * @brief Get name of interface
   *
   * @return const std::string&
   */
  [[nodiscard]] const std::string & name() const noexcept { return name_; }

  /**
   * @brief Interface is ready
   *
   * @return true - Ready
   * @return false - Otherwise
   */
  [[nodiscard]] virtual bool ready() const = 0;

  /**
   * @brief Number of elements in the interface
   *
   * @return std::size_t - Number of elements
   */
  [[nodiscard]] virtual std::size_t data_size() const = 0;

  /**
   * @brief Prepare the data for a given index
   *
   * @param idx - Index of the data to prepare
   * @return true - Successful
   * @return false - Otherwise
   */
  virtual bool prepare(std::size_t idx) = 0;

  /**
   * @brief Play the data for a given index
   *
   * @param idx - Index of the data to play
   * @return true - Successful
   * @return false - Otherwise
   */
  virtual bool play(std::size_t idx) = 0;

  /**
   * @brief Destroy the Play Data Interface Base object
   *
   */
  virtual ~PlayDataInterfaceBase() = default;

protected:
  std::string name_;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__PLAY_DATA_INTERFACE_BASE_HPP_
