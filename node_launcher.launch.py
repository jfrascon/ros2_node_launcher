import json
import logging
import re
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Tuple
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context: LaunchContext, *args, **kwargs) -> Optional[List[LaunchDescriptionEntity]]:
    logger = logging.getLogger(__name__)
    lc_config_file = LaunchConfiguration("config_file")

    try:
        config_file = Path(lc_config_file.perform(context))
    except Exception as e:
        logger.error(f"The parameter 'config_file' is not set: {e}")
        raise

    try:
        node_config, namespace = load_ros_node_config(config_file)
    except Exception as e:
        logger.error(f"Error while loading node configuration: {e}")
        raise

    node_name = next(iter(node_config))
    ros_parameters = node_config[node_name]["ros__parameters"]

    # print(namespace)
    # print(node_name)
    # print(ros_parameters)

    ros_execution = ros_parameters.get("ros_execution")

    if not isinstance(ros_execution, dict):
        raise ValueError("The field 'ros_execution' is required and must be a dictionary in the YAML configuration")

    for key in ("node_package", "node_executable"):
        if not ros_execution.get(key):
            raise ValueError(f"The field '{key}' is required in the 'ros_execution' block of the YAML configuration")

    output = ros_execution.get("output", "screen")
    emulate_tty = ros_execution.get("emulate_tty", False)
    respawn = ros_execution.get("respawn", False)
    respawn_delay = ros_execution.get("respawn_delay", 0)

    remapping_list: List[Tuple[str, str]] = []

    if "ros_remappings" in ros_parameters and ros_parameters["ros_remappings"] is not None:
        remapping_list = get_remappings(ros_parameters["ros_remappings"])
        # print(remapping_list)

    # Parameters are passed as a list, with each element either a yaml file that contains
    # parameter rules or a dictionary that specifies parameter rules.

    expand_filenames(ros_parameters)

    # print(ros_parameters)

    node = Node(
        package=ros_execution["node_package"],
        executable=ros_execution["node_executable"],
        namespace=namespace,
        name=node_name,
        parameters=[ros_parameters],
        remappings=remapping_list,
        output=output,
        emulate_tty=emulate_tty,
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    actions: List[LaunchDescriptionEntity] = [node]

    #  Optional: shut down launch if this node exits
    if ros_execution.get("on_node_exit_shutdown_system", False):
        shutdown_on_exit = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
        actions.append(shutdown_on_exit)

    return actions


def expand_filenames(yaml_parameters: Dict[str, Any]) -> None:
    """
    Recursively traverses a ROS parameter dictionary and expands any values that use
    'package://' or 'file://' URI-style path patterns into absolute file system paths.

    The function modifies the input dictionary in-place.

    Recognized patterns:
      - 'package://<package>/a/relative/path/file.txt'
        is expanded to:
        '/absolute/path/to/<package>/a/relative/path/file.txt'

      - 'file:///absolute/path/to/a/file.txt'
        is expanded to:
        '/absolute/path/to/a/file.txt'

    This allows ROS configuration files to avoid hardcoding absolute paths,
    improving portability across systems.

    Parameters
    ----------
    yaml_parameters : dict
        Dictionary representing ROS 2 parameters, typically under 'ros__parameters'.
    """
    if yaml_parameters is None or not isinstance(yaml_parameters, dict):
        return

    pattern1 = r"package://([^/]+)/(.+)"
    pattern2 = r"file://(/.+)"

    for param_id, value in yaml_parameters.items():
        if isinstance(value, dict):
            expand_filenames(value)
            continue

        if not isinstance(value, str):
            continue

        match1 = re.search(pattern1, value)

        if match1 is not None:
            package = get_package_share_directory(match1.group(1))
            relative_path = match1.group(2)
            yaml_parameters[param_id] = os.path.join(package, relative_path)
            continue

        match2 = re.search(pattern2, value)

        if match2 is not None:
            yaml_parameters[param_id] = match2.group(1)


def get_node_config_and_ns(yaml_config: dict, namespace: str = "") -> Tuple[dict, str]:
    """
    Recursively traverses a hierarchical YAML configuration structure to locate the node definition.

    The structure is expected to contain exactly one key per level, forming nested namespaces, until
    reaching the node definition which must contain a 'ros__parameters' field.

    Parameters
    ----------
    yaml_config : dict
        The parsed YAML configuration as a nested dictionary.
    namespace : str, optional
        The current namespace being accumulated during recursion. Defaults to an empty string.

    Returns
    -------
    Tuple[dict, str]
        A tuple where the first element is the dictionary containing the node definition
        (i.e., the level with 'ros__parameters'), and the second is the accumulated namespace string.

    Raises
    ------
    ValueError
        If the input is not a dictionary, if any level contains more than one key,
        or if the structure is invalid and does not contain a 'ros__parameters' field.

    Example:
    namespace_1:
        namespace_2:
            ...
                namespace_n:
                    node_name:
                        ros__parameters:
                            param_1:
                            param_2:
                            ...
                            param_n
    """

    if not isinstance(yaml_config, dict):
        raise ValueError("YAML configuration must be a dictionary at every level")

    keys = list(yaml_config.keys())

    if len(keys) != 1:  # only one key allowed until we get the params under ros__parameters field
        raise ValueError(f"Expected exactly one key per level, but found {len(keys)} keys: {keys}")

    current_key = keys[0]  # there is only one key at this level
    inner_yaml_config = yaml_config[current_key]

    if not isinstance(inner_yaml_config, dict):
        raise ValueError(f"Expected a dictionary under key '{current_key}', but got {type(inner_yaml_config).__name__}")

    if "ros__parameters" in inner_yaml_config:
        if not isinstance(inner_yaml_config["ros__parameters"], dict):
            raise ValueError(f"'ros__parameters' under node '{current_key}' must be a dictionary.")

        return yaml_config, namespace

    next_namespace = f"{namespace}/{current_key}" if namespace else current_key
    return get_node_config_and_ns(inner_yaml_config, next_namespace)


def get_remappings(ros_remappings: Optional[Dict[str, Any]]) -> List[Tuple[str, str]]:
    """
    Parses remapping entries from a ROS 2 YAML configuration and returns a list of (from, to) tuples.

    This function supports remappings under the following sections:

    Example YAML structure:
        ros_remappings:
          topics:
            # subscribers
            - "from_1:to_1"
            - "from_2:to_2"
            # publishers
            - "from_i:to_i"
          services:
            # service servers
            - "from_l:to_l"
            # service clients
            - "from_m:to_m"
          actions:
            # action servers
            - "from_p:to_p"
            # action clients
            - "from_q:to_q"

    The 'topics' and 'services' entries are parsed directly into (from, to) remapping tuples.

    For the 'actions' section, each string of the form "from:to" is expanded into the
    five standard ROS 2 action interface remappings:

        [
         (from/_action/feedback,    to/_action/feedback),
         (from/_action/status,      to/_action/status),
         (from/_action/cancel_goal, to/_action/cancel_goal),
         (from/_action/get_result,  to/_action/get_result),
         (from/_action/send_goal,   to/_action/send_goal),
         ...
        ]

    Parameters
    ----------
    ros_remappings : dict or None
        Dictionary containing optional 'topics', 'services', and 'actions' remapping entries.

    Returns
    -------
    List[Tuple[str, str]]
        A list of remapping rules as (from, to) tuples, suitable for the 'remappings' field of a ROS 2 Node.

    Raises
    ------
    ValueError
        If the structure is malformed or any remapping entry is invalid.
    """
    if ros_remappings is None:
        return []

    if not isinstance(ros_remappings, dict):
        raise ValueError("Expected 'ros_remappings' to be a dictionary.")

    remapping_list = []

    def parse_section(section_name: str):
        section = ros_remappings.get(section_name)
        if section is None:
            return []

        if not isinstance(section, list):
            raise ValueError(f"Expected a list for '{section_name}' remappings.")

        result = []
        for item in section:
            if not isinstance(item, str):
                raise ValueError(f"Each remapping in '{section_name}' must be a string, got {type(item).__name__}")
            parts = item.split(":")
            if len(parts) != 2:
                raise ValueError(f"Invalid format in '{section_name}': '{item}' (expected 'from:to')")
            from_topic, to_topic = parts[0].strip(), parts[1].strip()
            if not from_topic or not to_topic:
                raise ValueError(f"Empty 'from' or 'to' field in '{section_name}' remapping: '{item}'")
            result.append((from_topic, to_topic))

        return result

    remapping_list.extend(parse_section("topics"))
    remapping_list.extend(parse_section("services"))

    if "actions" in ros_remappings:
        remapping_list.extend(get_remappings_for_actions(ros_remappings["actions"]))

    return remapping_list


def get_remappings_for_actions(actions_yaml: List[str]) -> List[Tuple[str, str]]:
    """
    Expands action remapping strings into full topic remappings for each ROS 2 action subinterface.

    Each item in actions_yaml must be a string formatted as "from:to".
    For each of these pairs, the function generates five remappings corresponding to the
    standard action topics in ROS 2:

        [
         (from/_action/feedback,    to/_action/feedback),
         (from/_action/status,      to/_action/status),
         (from/_action/cancel_goal, to/_action/cancel_goal),
         (from/_action/get_result,  to/_action/get_result),
         (from/_action/send_goal,   to/_action/send_goal),
         ...
        ]

    Parameters
    ----------
    actions_yaml : List[str]
        A list of remapping strings of the form "from_action:to_action".

    Returns
    -------
    List[Tuple[str, str]]
        Expanded list of remapping tuples for all action-related subtopics.

    Raises
    ------
    ValueError
        If any remapping string is malformed or missing the expected 'from:to' format.
    """

    if actions_yaml is None:
        return []

    if not isinstance(actions_yaml, list):
        raise ValueError("Expected a list for 'actions' remappings.")

    suffixes = [
        "/_action/feedback",
        "/_action/status",
        "/_action/cancel_goal",
        "/_action/get_result",
        "/_action/send_goal",
    ]

    remapping_list = []

    # Reference: https://github.com/ros2/ros2/issues/1312#issuecomment-1705521109
    # item is a dictionary, each {from: "from_i", to: "to_i"}
    for item in actions_yaml:
        if not isinstance(item, str):
            raise ValueError(f"Each action remapping must be a string, got {type(item).__name__}")

        parts = item.split(":")

        if len(parts) != 2:
            raise ValueError(f"Invalid format in action remapping: '{item}' (expected 'from:to')")

        from_prefix, to_prefix = parts[0].strip(), parts[1].strip()

        if not from_prefix or not to_prefix:
            raise ValueError(f"Empty 'from' or 'to' in action remapping: '{item}'")

        for suffix in suffixes:
            remapping_list.append((from_prefix + suffix, to_prefix + suffix))

    return remapping_list


def load_json_config(json_file: Path) -> Any:
    """
    Loads a JSON configuration file and returns its content as a Python object.

    Parameters
    ----------
    json_file : Path
        Path to the JSON file.

    Returns
    -------
    Any
        The parsed JSON content (can be a dict, list, or primitive type).

    Raises
    ------
    ValueError
        If the path is None, does not exist, is not a file, or contains no usable content.
    json.JSONDecodeError
        If the JSON content is invalid.
    """
    if json_file is None:
        raise ValueError("No path to JSON configuration file was provided.")

    if not json_file.exists():
        raise ValueError(f"The JSON configuration file '{json_file}' does not exist.")

    if not json_file.is_file():
        raise ValueError(f"The path '{json_file}' is not a file.")

    with json_file.open("r") as file:
        data = json.load(file)

    if data is None:
        raise ValueError(f"The JSON configuration file '{json_file}' is empty or contains no usable content.")

    return data


def load_ros_node_config(yaml_file: Path) -> Tuple[dict, str]:
    """
    Loads and validates a ROS node configuration YAML file.

    Ensures the structure contains a single node with 'ros__parameters'.

    Parameters
    ----------
    yaml_file : Path
        Path to the YAML file.

    Returns
    -------
    Tuple[dict, str]
        Tuple with the node configuration dictionary and its namespace.

    Raises
    ------
    Exception
        If loading or structure validation fails.
    """
    yaml_config = load_yaml_config(yaml_file)
    return get_node_config_and_ns(yaml_config)


def load_yaml_config(yaml_file: Path) -> dict:
    """
    Loads a YAML configuration file and returns its content as a dictionary.

    Parameters
    ----------
    yaml_file : Path
        Path to the YAML file.

    Returns
    -------
    dict
        The parsed YAML content, expected to be a dictionary suitable for ROS node configuration.

    Raises
    ------
    ValueError
        If the path is None, does not exist, is not a file, is empty, or does not contain a dictionary.
    yaml.YAMLError
        If the YAML content is invalid.
    """
    if yaml_file is None:
        raise ValueError("No path to YAML configuration file was provided.")

    if not yaml_file.exists():
        raise ValueError(f"The YAML configuration file '{yaml_file}' does not exist.")

    if not yaml_file.is_file():
        raise ValueError(f"The path '{yaml_file}' is not a file.")

    with yaml_file.open("r") as file:
        data = yaml.safe_load(file)

    if data is None:
        raise ValueError(f"The YAML configuration file '{yaml_file}' is empty or contains no usable content.")

    if not isinstance(data, dict):
        raise ValueError(f"The YAML configuration file '{yaml_file}' must contain a dictionary at the top level.")

    return data


def generate_launch_description():

    # If a default value es added to the function DeclareLaunchArgument, then the expression
    # DeclareLaunchArgument(config_file, <default_value>, description=...) must be assigned to a variable
    # on the left-hand-side.
    DeclareLaunchArgument("config_file", description="Configuration file")

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=configure))

    return ld
