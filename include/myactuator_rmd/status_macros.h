#ifndef MYACTUATOR_RMD_STATUS_MACROS_H_
#define MYACTUATOR_RMD_STATUS_MACROS_H_

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace myactuator_rmd {

// 상태 반환 매크로: 주어진 표현식이 오류 상태를 반환하면 해당 오류를 즉시 반환
#define RETURN_IF_ERROR(expr)                                    \
  do {                                                           \
    const absl::Status _status = (expr);                         \
    if (!_status.ok()) {                                         \
      return _status;                                            \
    }                                                            \
  } while (0)

// StatusOr 처리 매크로: 표현식이 오류를 포함하는 StatusOr를 반환하면 오류를 반환하고,
// 그렇지 않으면 StatusOr의 값을 지정된 변수에 할당
#define ASSIGN_OR_RETURN(lhs, expr)                                 \
  do {                                                              \
    auto _statusor = (expr);                                        \
    if (!_statusor.ok()) {                                          \
      return _statusor.status();                                    \
    }                                                               \
    lhs = std::move(_statusor).value();                             \
  } while (0)

}  // namespace myactuator_rmd

#endif  // MYACTUATOR_RMD_STATUS_MACROS_H_ 