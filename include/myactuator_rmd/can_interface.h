#ifndef V161_MOTOR_CONTROL__CAN_INTERFACE_HPP
#define V161_MOTOR_CONTROL__CAN_INTERFACE_HPP

#include <array>
#include <chrono>
#include <memory>  // for std::unique_ptr
#include <string>
#include <vector>

#include <cstdint>

#include "myactuator_rmd/can/exceptions.h"
#include "myactuator_rmd/can/node.h"
#include "absl/status/status.h"

namespace v161_motor_control {

/**
 * @brief CAN 통신 인터페이스를 제공하는 클래스
 *
 * 이 클래스는 MyActuator RMD 시리즈 모터와의 CAN 통신을 위한 저수준 인터페이스를 제공합니다.
 * 프레임 송수신, 필터 설정 등의 기능을 담당하며, 내부적으로 SocketCAN을 사용합니다.
 * 모든 통신 오류는 absl::Status를 통해 반환됩니다.
 */
class CanInterface {
 public:
  /**
   * @brief CAN 인터페이스를 초기화하는 생성자
   * 
   * @param ifname CAN 인터페이스 이름 (예: "can0", "vcan0")
   * @param send_timeout_us 송신 타임아웃 (마이크로초 단위, 기본값: 1초)
   * @param receive_timeout_us 수신 타임아웃 (마이크로초 단위, 기본값: 1초)
   * 
   * @note 생성자에서 CAN 인터페이스 연결에 실패할 경우 예외가 발생할 수 있습니다.
   */
  CanInterface(const std::string& ifname,
               long send_timeout_us = 1000000,    // 1s
               long receive_timeout_us = 1000000  // 1s
  );
  ~CanInterface();

  /**
   * @brief CAN 프레임을 송신합니다.
   * 
   * @param can_id 목표 CAN ID
   * @param data 송신할 8바이트 데이터
   * 
   * @return 성공 또는 실패를 나타내는 absl::Status
   * @retval absl::OkStatus() 프레임 송신 성공
   * @retval absl::UnavailableError CAN 인터페이스 사용 불가
   * @retval absl::DeadlineExceededError 송신 타임아웃 발생
   * @retval absl::InternalError 내부 송신 오류 발생
   */
  absl::Status sendFrame(uint32_t can_id, const std::array<uint8_t, 8>& data);

  /**
   * @brief 특정 CAN ID로부터 프레임을 수신합니다. 타임아웃까지 대기합니다.
   * 
   * @param expected_can_id 수신을 기대하는 CAN ID
   * @param data_out 수신 데이터를 저장할 배열 (출력 매개변수)
   * 
   * @return 성공 또는 실패를 나타내는 absl::Status
   * @retval absl::OkStatus() 프레임 수신 성공
   * @retval absl::UnavailableError CAN 인터페이스 사용 불가
   * @retval absl::DeadlineExceededError 수신 타임아웃 발생
   * @retval absl::InternalError 내부 수신 오류 발생
   * @retval absl::InvalidArgumentError 예상하지 않은 CAN ID 수신
   */
  absl::Status receiveFrame(uint32_t expected_can_id, std::array<uint8_t, 8>& data_out);

  /**
   * @brief CAN 수신 필터를 설정하여 특정 ID만 수신하도록 합니다.
   * 
   * @param ids 필터링할 CAN ID 벡터
   * 
   * @return 성공 또는 실패를 나타내는 absl::Status
   * @retval absl::OkStatus() 필터 설정 성공
   * @retval absl::UnavailableError CAN 인터페이스 사용 불가
   * @retval absl::InvalidArgumentError 잘못된 필터 설정 요청
   * @retval absl::InternalError 내부 필터 설정 오류 발생
   */
  absl::Status setReceiveFilters(const std::vector<uint32_t>& ids);

  // Disable copy and move constructor/assignment operators (because of
  // unique_ptr member)
  CanInterface(const CanInterface&) = delete;
  CanInterface& operator=(const CanInterface&) = delete;
  CanInterface(CanInterface&&) = delete;
  CanInterface& operator=(CanInterface&&) = delete;

 private:
  std::unique_ptr<myactuator_rmd::can::Node> can_node_;
  std::string ifname_;
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__CAN_INTERFACE_HPP
