# TreeExecutionServer

This base class is used to implement a Behavior Tree executor wrapped inside a `rclcpp_action::Server`.

Users are expected to create a derived class to improve its functionalities, but often it can be used
out of the box directly.

Further, the terms "load" will be equivalent to "register into the `BT::BehaviorTreeFactory`".

The `TreeExecutionServer` offers the following features:

- Configurable using ROS parameters (see below).
- Load Behavior Trees definitions (XML files) from a list of folders.
- Load BT plugins from a list of folders. These plugins may depend or not on ROS.
- Invoke the execution of a Tree from an external ROS Node, using `rclcpp_action::Client`.

Furthermore, the user can customize it to:

- Register custom BT Nodes directly (static linking).
- Attach additional loggers. The **Groot2** publisher will be attached by default.
- Use the "global blackboard", a new idiom/pattern explained in [this tutorial](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/examples/t16_global_blackboard.cpp).
- Customize the feedback of the `rclcpp_action::Server`.

## Customization points

These are the virtual method of `TreeExecutionServer` that can be overridden by the user.

### void onTreeCreated(BT::Tree& tree)

Callback invoked when a tree is created; this happens after `rclcpp_action::Server` receive a command from a client.

It can be used, for instance, to initialize a logger or the global blackboard.

###  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)

Called at the beginning, after all the plugins have been loaded.

It can be used to register programmatically more BT.CPP Nodes.

### std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status)

Used to do something after the tree was ticked, in its execution loop.

If this method returns something other than `std::nullopt`, the tree
execution is interrupted an the specified `BT::NodeStatus` is returned to the `rclcpp_action::Client`.

### void onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)

Callback when the tree execution reaches its end.

This happens if:

1. Ticking the tree returns SUCCESS/FAILURE
2. The `rclcpp_action::Client` cancels the action.
3. Callback `onLoopAfterTick`cancels the execution.

Argument `was_cancelled`is true in the 1st case, false otherwise.

### std::optional<std::string> onLoopFeedback()

This callback is invoked after `tree.tickOnce` and `onLoopAfterTick`.

If it returns something other than `std::nullopt`, the provided string will be
sent as feedback to the `rclcpp_action::Client`.



## ROS Parameters

Documentation for the parameters used by the `TreeExecutionServer` can be found [here](bt_executor_parameters.md).

If the parameter documentation needs updating you can regenerate it with:
```bash
generate_parameter_library_markdown --input_yaml src/bt_executor_parameters.yaml --output_markdown_file bt_executor_parameters.md
```
