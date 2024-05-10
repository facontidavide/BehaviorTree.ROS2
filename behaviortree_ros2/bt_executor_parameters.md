# Bt Server Parameters

Default Config
```yaml
bt_server:
  ros__parameters:
    action_name: bt_execution
    behavior_trees: '{}'
    groot2_port: 1667.0
    plugins: '{}'
    ros_plugins_timeout: 1000.0
    tick_frequency: 100.0

```

## action_name

The name the Action Server takes requests from

* Type: `string`
* Default Value: "bt_execution"
* Read only: True

## tick_frequency

Frequency in Hz to tick() the Behavior tree at

* Type: `int`
* Default Value: 100
* Read only: True

*Constraints:*
 - parameter must be within bounds 1

## groot2_port

Server port value to publish Groot2 messages on

* Type: `int`
* Default Value: 1667
* Read only: True

*Constraints:*
 - parameter must be within bounds 1

## plugins

List of 'package_name/subfolder' containing BehaviorTree plugins to load into the factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates

## ros_plugins_timeout

Timeout (ms) used in BT::RosNodeParams

* Type: `int`
* Default Value: 1000

*Constraints:*
 - parameter must be within bounds 1

## behavior_trees

List of 'package_name/subfolder' containing SubTrees to load into the BehaviorTree factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates

## tree_on_initialization

The name of the behavior tree to launch on intialization. Defaults to no behavior tree launched on initialization

* Type: `string`
* Default Value: ""
* Read only: True
