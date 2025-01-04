# ssl_ros_bridge

This repository provides ROS 2 packages containing utilities for connecting your ROS system to the RoboCup Small Size League (SSL) standard systems, such as ssl-vision and the Game Controller.

## Installation

This repo is designed to be cloned into a colcon workspace. You can clone this repo alongside your own code or as a submodule within your own repository.

## Usage

The intended usage of these packages is to configure and run the nodes in the ssl_ros_bridge package alongside your own nodes. Each node is provided as a component with a standalone executable available. Either can be used.

For most teams, in most scenarios, the only parameter that needs to be configured is the team name. Automatic discovery and multi-interface multicast listening will handle most common network setups.

## Packages

### ssl_league_protobufs

This package includes the league-defined protobuf files and builds them into a library available for other packages.

### ssl_league_msgs

This package defines ROS messages which closely mirror the league protobufs.

#### Optional Fields

The SSL League protobufs make extensive use of optional fields, which are unfortunately not currently supported in ROS messages. ssl_ros_bridge uses array field types to hold optionals. These fields will either hold one or zero elements. This does introduce some ambiguity between optional fields and actual array fields. Users should refer to the corresponding protobuf definitions for clarity.

_Note:_ ROS fields do support bounded-size array types. We could enforce the "zero or one" behavior with these size limits, but this has caused problems with some ROS tools in the past (ie. PlotJuggler) which don't parse the message definitions correctly with these limits in place.

#### Recursive Message Definitions

The league protobufs include a few recursive definitions. For example, the `GameEvent` message may hold a `MultipleFouls` message which itself holds an array of `GameEvent` messages. ROS does not support recursive message types. In practice, most teams will not need these fields, so ssl_ros_bridge omits them from the ROS messages.

### ssl_ros_bridge_msgs

This package defines interfaces used by the bridge nodes which are not mirrors of league messages. These are mostly services users can use to send requests up to league software.

### ssl_ros_bridge

This package provides the nodes which implement the bridge between league software and ROS systems.

#### vision_bridge

This node listens for the multicast vision messages sent by ssl-vision and publishes them into ROS.

##### Published Topics

* ~/vision_messages
   * Type: [ssl_league_msgs/msg/VisionWrapper](ssl_league_msgs/msg/vision/VisionWrapper.msg)
   * Contains vision data including robot detections, ball detections, and field geometry.

##### Parameters

* ssl_vision_ip
  * Type: string
  * Default: "224.5.23.2"
  * The multicast group address to listen for.
* ssl_vision_port
  * Type: int
  * Default: 10020
  * The multicast group port to listen for.
* net_interface_address
  * Type: string
  * Default: empty
  * When empty, the node will join the multicast group on all interfaces. When set to an IP address associated with one of your machine's network interfaces, the node will only join the multicast group on that interface.

#### game_controller_bridge

This node listens for multicast game controller messages and republishes them into ROS.

##### Automatic Game Controller Discovery

The team client node needs to know the IP address fo the game controller server to establish its connection. This address is often different at different fields even within the same event and can be tedious to keep configured correctly. The game controller bridge node will automatically configure the team client node with the correct address based on the sender of the multicast messages. The game controller bridge will only reconfigure the team client node if it is not already connected.

##### Published Topics

* ~/referee_messages
  * Type: [ssl_league_msgs/msg/Referee](ssl_league_msgs/msg/game_controller/Referee.msg)
  * Contains the latest information from the game controller.

##### Subscribed Topics

* /team_client_node/connection_status
  * Type: [ssl_ros_bridge_msgs/msg/TeamClientConnectionStatus](ssl_ros_bridge_msgs/msg/TeamClientConnectionStatus.msg)
  * Used for automatic game controller discovery.

##### Service Clients

* /team_client_node/reconnect
  * Type: [ssl_ros_bridge_msgs/srv/ReconnectTeamClient](ssl_ros_bridge_msgs/srv/ReconnectTeamClient.srv)
  * Used for automatic game controller discovery.

##### Parameters

* multicast.address
  * Type: string
  * Default: "224.5.23.1"
  * The multicast group address to listen for.
* multicast.port
  * Type: int
  * Default: 10003
  * The multicast group port to listen for.
* net_interface_address
  * Type: string
  * Default: empty
  * When empty, the node will join the multicast group on all interfaces. When set to an IP address associated with one of your machine's network interfaces, the node will only join the multicast group on that interface.

#### team_client

This node connects as a team client to the game controller server. ROS services can then be used to send requests to the game controller, such as changing the goal keeper ID number.

_Note_: Encrypted connections are not currently supported.

##### Published Topics

* ~/connection_status
  * Type: [ssl_ros_bridge_msgs/msg/TeamClientConnectionStatus](ssl_ros_bridge_msgs/msg/TeamClientConnectionStatus.msg)
  * Contains the connection status and ping time of the team client to the game controller server.

##### Services

* ~/set_desired_keeper
  * Type: [ssl_ros_bridge_msgs/srv/](ssl_ros_bridge_msgs/srv/SetDesiredKeeper.srv)
  * Sends a request to the game controller to change your team's keeper ID.
* ~/substitute_bot
  * Type: [ssl_ros_bridge_msgs/srv/](ssl_ros_bridge_msgs/srv/SubstituteBot.srv)
  * Sends a request to the game controller to substitute a bot.
* ~/reconnect
  * Type: [ssl_ros_bridge_msgs/srv/](ssl_ros_bridge_msgs/srv/ReconnectTeamClient.srv)
  * Reconnects the team client to the game controller.
  * _Note_: If the address field of the request is empty, the team client node will reconnect to the same server it was previously configured for. If the address field of the request is not empty, the `gc_ip_address` parameter will be overwritten with this new address.
* ~/set_advantage_choice
  * Type: [ssl_ros_bridge_msgs/srv/](ssl_ros_bridge_msgs/srv/SetTeamAdvantageChoice.srv)
  * Sends a request to set the team's advantage choice to the game controller.

##### Parameters

* gc_ip_address
  * Type: string
  * Default: empty
  * The address of the game controller server. Leave empty to let the game controller bridge discovery the address automatically.
* gc_port
  * Type: int
  * Default: 10008
  * The port on the game controller server the team client will connect to.
* team_name
  * Type: string
  * Default: "A-Team"
  * The name of the team the client will try to connect as.
* team_color
  * Type: string
  * Default: "auto"
  * The team color to connect as. Only relevant if a team is playing against itself and the identical team names need to be disambiguated.

## Contributing

See [our contributing guidelines](blob/master/CONTRIBUTING.md) 
