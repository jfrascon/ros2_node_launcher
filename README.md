# ROS 2 Node Launcher from Hierarchical YAML

This script provides a flexible and reusable way to launch a ROS 2 node from a hierarchical YAML configuration file.  
It is designed to be copied directly into your bringup package and used from your `launch` files — no installation or packaging required.

---

## Features

- Launch any ROS 2 node from a structured YAML config
- Support for nested namespaces
- Remappings directly from YAML: topics, services, and actions
- Automatic path resolution for `package://` and `file://` URIs
- Node execution options: respawn, emulate TTY, shutdown on exit
- Drop-in integration: no setup.py, no pip install

---

## YAML Structure

```yaml
my_ns:
  sub_ns:
    my_node:
      ros__parameters:
        param_1: true
        param_2: 42
        camera_config: package://my_robot_description/config/camera.yaml
        static_file: file:///home/user/data/fixed_params.yaml
        ros_remappings:
          topics:
            - "/camera/image_raw:/namespace/image_raw"
          services:
            - "/get_status:/namespace/get_status"
          actions:
            - "/navigate_to_pose:/namespace/navigate_to_pose"
        ros_execution:
          node_package: my_package
          node_executable: my_node_exec
          output: screen
          emulate_tty: true
          respawn: true
          respawn_delay: 2.0
          on_node_exit_shutdown_system: true
````

This structure defines:

* A nested namespace: `my_ns/sub_ns`
* A node named `my_node`
* Parameters under `ros__parameters`
* Remappings for topics, services, and actions
* Execution behavior under `ros_execution`

---

## Remapping behavior

Remappings are grouped by interface type:

* **topics**: list of `"from:to"` strings
* **services**: same format as topics
* **actions**: automatically expanded into five ROS 2 subtopics:

  * `_action/feedback`
  * `_action/status`
  * `_action/cancel_goal`
  * `_action/get_result`
  * `_action/send_goal`

---

## Path resolution

This tool resolves paths using common URI-style prefixes:

* `package://my_pkg/config/my_file.yaml`
  → resolved to `${AMENT_PREFIX_PATH}/share/my_pkg/config/my_file.yaml`

* `file:///home/user/some/path/file.yaml`
  → resolved directly to `/home/user/some/path/file.yaml`

This is useful for loading external configuration files in parameters.

---

## Contact

For any questions or suggestions, please reach out to:

    Juan Francisco Rascon
    Email: jfrascon@gmail.com
