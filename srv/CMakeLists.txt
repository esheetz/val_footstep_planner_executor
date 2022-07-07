add_service_files(
  DIRECTORY
  ./
  FILES
  PlanToStance.srv
  PlanToWaypoint.srv
  ExecuteToStance.srv
  ExecuteToWaypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  controller_msgs
)

message ("generated service files!")
