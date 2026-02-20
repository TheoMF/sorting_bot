#ifndef SORTING_BOT_DATA_STRUCTURES_HPP_
#define SORTING_BOT_DATA_STRUCTURES_HPP_

#include <optional>
#include <typeindex>
#include <variant>

#include "builtin_interfaces/msg/time.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace sorting_bot {

struct Detection {
  std::string parent_frame, frame;
  bool is_in_fov;
  std::optional<builtin_interfaces::msg::Time> last_stamp;
  std::optional<pinocchio::SE3> in_parent_M_frame;

  Detection() {
    parent_frame = "";
    frame = "";
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_parent_M_frame = std::nullopt;
  }

  Detection(const std::string &arg_parent_frame, const std::string &arg_frame)
      : parent_frame(arg_parent_frame), frame(arg_frame) {
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_parent_M_frame = std::nullopt;
  }
};

enum ActionType { NONE, WAIT, MOVE_GRIPPER, MOVE_BASE, MOVE_ARM, SEARCH_OBJECT, SEARCH_BOX };

struct Action {
  ActionType type;
  int id;
  std::variant<std::monostate, double, Eigen::VectorXd> value;

  const char *get_value_type_name() const {
    return std::visit([](auto &v) -> std::type_index { return typeid(v); }, value).name();
  }
};

inline std::string get_action_type_as_string(const ActionType &action_type) {
  std::string action_name;
  switch (action_type) {
  case NONE:
    action_name = "NONE";
    break;
  case WAIT:
    action_name = "WAIT";
    break;
  case MOVE_GRIPPER:
    action_name = "MOVE_GRIPPER";
    break;
  case MOVE_BASE:
    action_name = "MOVE_BASE";
    break;
  case MOVE_ARM:
    action_name = "MOVE_ARM";
    break;
  case SEARCH_OBJECT:
    action_name = "SEARCH_OBJECT";
    break;
  case SEARCH_BOX:
    action_name = "SEARCH_BOX";
    break;
  }
  return action_name;
}

} // namespace sorting_bot

#endif