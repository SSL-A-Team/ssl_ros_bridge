cmake_minimum_required(VERSION 3.5)
project(ssl_league_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(GC_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/game_controller)
set(VISION_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vision)
set(SIMULATOR_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/simulator)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${GC_MSG_DIR}:msg/ControllerReply.msg
  ${GC_MSG_DIR}:msg/GameEvent.msg
  ${GC_MSG_DIR}:msg/GameEventProposalGroup.msg
  ${GC_MSG_DIR}:msg/Referee.msg
  ${GC_MSG_DIR}:msg/TeamInfo.msg

  ${GC_MSG_DIR}/common:msg/Team.msg
  ${GC_MSG_DIR}/common:msg/RobotId.msg
  ${GC_MSG_DIR}/common:msg/Division.msg

  ${GC_MSG_DIR}/game_events:msg/DefenderInDefenseArea.msg
  ${GC_MSG_DIR}/game_events:msg/TooManyRobots.msg
  ${GC_MSG_DIR}/game_events:msg/BotSubstitution.msg
  ${GC_MSG_DIR}/game_events:msg/EmergencyStop.msg
  ${GC_MSG_DIR}/game_events:msg/UnsportingBehaviorMajor.msg
  ${GC_MSG_DIR}/game_events:msg/AttackerTooCloseToDefenseArea.msg
  ${GC_MSG_DIR}/game_events:msg/BotInterferedPlacement.msg
  ${GC_MSG_DIR}/game_events:msg/KeeperHeldBall.msg
  ${GC_MSG_DIR}/game_events:msg/BotCrashUnique.msg
  ${GC_MSG_DIR}/game_events:msg/Prepared.msg
  ${GC_MSG_DIR}/game_events:msg/AttackerTouchedOpponentInDefenseArea.msg
  ${GC_MSG_DIR}/game_events:msg/DefenderTooCloseToKickPoint.msg
  ${GC_MSG_DIR}/game_events:msg/PlacementFailed.msg
  ${GC_MSG_DIR}/game_events:msg/DefenderInDefenseAreaPartially.msg
  ${GC_MSG_DIR}/game_events:msg/BotDribbledBallTooFar.msg
  ${GC_MSG_DIR}/game_events:msg/UnsportingBehaviorMinor.msg
  ${GC_MSG_DIR}/game_events:msg/PenaltyKickFailed.msg
  ${GC_MSG_DIR}/game_events:msg/KickTimeout.msg
  ${GC_MSG_DIR}/game_events:msg/BotCrashDrawn.msg
  ${GC_MSG_DIR}/game_events:msg/ChippedGoal.msg
  ${GC_MSG_DIR}/game_events:msg/NoProgressInGame.msg
  ${GC_MSG_DIR}/game_events:msg/PlacementSucceeded.msg
  ${GC_MSG_DIR}/game_events:msg/BotKickedBallTooFast.msg
  ${GC_MSG_DIR}/game_events:msg/MultiplePlacementFailures.msg
  ${GC_MSG_DIR}/game_events:msg/ChallengeFlag.msg
  ${GC_MSG_DIR}/game_events:msg/BallLeftField.msg
  ${GC_MSG_DIR}/game_events:msg/Goal.msg
  ${GC_MSG_DIR}/game_events:msg/BotHeldBallDeliberately.msg
  ${GC_MSG_DIR}/game_events:msg/AttackerTouchedBallInDefenseArea.msg
  ${GC_MSG_DIR}/game_events:msg/BotTooFastInStop.msg
  ${GC_MSG_DIR}/game_events:msg/AimlessKick.msg
  ${GC_MSG_DIR}/game_events:msg/IndirectGoal.msg
  ${GC_MSG_DIR}/game_events:msg/BotPushedBot.msg
  ${GC_MSG_DIR}/game_events:msg/MultipleFouls.msg
  ${GC_MSG_DIR}/game_events:msg/BotTippedOver.msg
  ${GC_MSG_DIR}/game_events:msg/AttackerDoubleTouchedBall.msg
  ${GC_MSG_DIR}/game_events:msg/BoundaryCrossing.msg
  ${GC_MSG_DIR}/game_events:msg/MultipleCards.msg
  ${GC_MSG_DIR}/game_events:msg/BotDroppedParts.msg
  ${GC_MSG_DIR}/game_events:msg/ChallengeFlagHandled.msg
  ${GC_MSG_DIR}/game_events:msg/ExcessiveBotSubstitution.msg

  ${VISION_MSG_DIR}:msg/VisionDetectionBall.msg
  ${VISION_MSG_DIR}:msg/VisionDetectionRobot.msg
  ${VISION_MSG_DIR}:msg/VisionDetectionFrame.msg

  ${VISION_MSG_DIR}:msg/VisionFieldCircularArc.msg
  ${VISION_MSG_DIR}:msg/VisionFieldLineSegment.msg
  ${VISION_MSG_DIR}:msg/VisionGeometryFieldSize.msg
  ${VISION_MSG_DIR}:msg/VisionGeometryCameraCalibration.msg
  ${VISION_MSG_DIR}:msg/VisionGeometryData.msg

  ${VISION_MSG_DIR}:msg/VisionWrapper.msg

  ${SIMULATOR_MSG_DIR}:msg/SimulatorControl.msg
  ${SIMULATOR_MSG_DIR}:msg/TeleportBallCommand.msg
  ${SIMULATOR_MSG_DIR}:msg/TeleportRobotCommand.msg

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
