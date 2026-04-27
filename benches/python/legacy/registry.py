# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

"""
Schema Registry for EdgeFirst Schemas.

This module provides functions to work with ROS2-style schema names:
- `from_schema()`: Get the message class for a schema name
- `schema_name()`: Get the schema name for a message class or instance

Schema names follow the ROS2 convention: `package/msg/TypeName`
(e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Pose`)

Example usage:

    from edgefirst.schemas import from_schema, schema_name, sensor_msgs

    # Get class from schema name
    ImageClass = from_schema("sensor_msgs/msg/Image")
    img = ImageClass()

    # Get schema name from class or instance
    name = schema_name(sensor_msgs.Image)  # "sensor_msgs/msg/Image"
    name = schema_name(img)                 # "sensor_msgs/msg/Image"
"""

from typing import Union

# Import all schema modules
from . import (
    builtin_interfaces,
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    nav_msgs,
    sensor_msgs,
    std_msgs,
)

# Type alias for schema classes
SchemaClass = type

# Registry mapping package names to their modules
_PACKAGE_MODULES = {
    "builtin_interfaces": builtin_interfaces,
    "edgefirst_msgs": edgefirst_msgs,
    "foxglove_msgs": foxglove_msgs,
    "geometry_msgs": geometry_msgs,
    "nav_msgs": nav_msgs,
    "sensor_msgs": sensor_msgs,
    "std_msgs": std_msgs,
}


def from_schema(schema: str) -> SchemaClass:
    """
    Get the message class for a given schema name.

    Args:
        schema: Schema name in ROS2 format (e.g., "sensor_msgs/msg/Image")

    Returns:
        The message class corresponding to the schema name

    Raises:
        ValueError: If the schema name format is invalid
        KeyError: If the package or type is not found

    Examples:
        >>> ImageClass = from_schema("sensor_msgs/msg/Image")
        >>> img = ImageClass()

        >>> PoseClass = from_schema("geometry_msgs/msg/Pose")
    """
    parts = schema.split("/")

    if len(parts) == 3:
        # Standard ROS2 format: package/msg/TypeName
        package, msg_marker, type_name = parts
        if msg_marker != "msg":
            raise ValueError(
                f"Invalid schema format '{schema}': "
                f"expected 'msg' but got '{msg_marker}'"
            )
    elif len(parts) == 2:
        # Short format: package/TypeName (used by pycdr2 typename)
        package, type_name = parts
    else:
        raise ValueError(
            f"Invalid schema format '{schema}': "
            f"expected 'package/msg/TypeName' or 'package/TypeName'"
        )

    # Look up the package module
    if package not in _PACKAGE_MODULES:
        raise KeyError(f"Unknown package '{package}' in schema '{schema}'")

    module = _PACKAGE_MODULES[package]

    # Use the module's get_type() function if available (hierarchical dispatch)
    if hasattr(module, 'get_type'):
        cls = module.get_type(type_name)
        if cls is not None:
            return cls
        raise KeyError(f"Unknown type '{type_name}' in package '{package}'")

    # Fallback: Look up the type directly in the module
    if not hasattr(module, type_name):
        raise KeyError(f"Unknown type '{type_name}' in package '{package}'")

    return getattr(module, type_name)


def schema_name(cls_or_instance: Union[SchemaClass, object]) -> str:
    """
    Get the ROS2 schema name for a message class or instance.

    Args:
        cls_or_instance: A message class or message instance

    Returns:
        Schema name in ROS2 format (e.g., "sensor_msgs/msg/Image")

    Raises:
        TypeError: If the argument is not a schema class or instance
        ValueError: If the class doesn't have a typename attribute

    Examples:
        >>> from edgefirst.schemas import sensor_msgs
        >>> schema_name(sensor_msgs.Image)
        'sensor_msgs/msg/Image'
        >>> img = sensor_msgs.Image()
        >>> schema_name(img)
        'sensor_msgs/msg/Image'
    """
    # Get the class if we have an instance
    if isinstance(cls_or_instance, type):
        cls = cls_or_instance
    else:
        cls = type(cls_or_instance)

    # Get the typename from pycdr2 IdlStruct metadata
    # pycdr2 stores typename in __idl_typename__ attribute
    if hasattr(cls, "__idl_typename__"):
        typename = cls.__idl_typename__
    else:
        raise ValueError(
            f"Class '{cls.__name__}' does not have a typename attribute. "
            "Ensure it inherits from IdlStruct with typename parameter."
        )

    # Convert pycdr2 format (package/TypeName) to ROS2 format (package/msg/Type)
    if "/" in typename:
        parts = typename.split("/")
        if len(parts) == 2:
            package, type_name = parts
            return f"{package}/msg/{type_name}"
        elif len(parts) == 3 and parts[1] == "msg":
            # Already in ROS2 format
            return typename
        else:
            raise ValueError(f"Unexpected typename format: '{typename}'")
    else:
        # No package prefix - try to infer from module
        module_name = cls.__module__
        if module_name.startswith("edgefirst.schemas."):
            package = module_name.split(".")[-1]
            return f"{package}/msg/{typename}"
        else:
            raise ValueError(
                f"Cannot determine package for type '{typename}' "
                f"from module '{module_name}'"
            )


def list_schemas(package: str = None) -> list[str]:
    """
    List all available schema names.

    Args:
        package: Optional package name to filter by (e.g., "sensor_msgs")

    Returns:
        List of schema names in ROS2 format

    Examples:
        >>> schemas = list_schemas("sensor_msgs")
        >>> "sensor_msgs/msg/Image" in schemas
        True
    """
    schemas = []
    packages = [package] if package else _PACKAGE_MODULES.keys()

    for pkg in packages:
        if pkg not in _PACKAGE_MODULES:
            continue

        module = _PACKAGE_MODULES[pkg]

        # Use the module's list_types() function if available
        if hasattr(module, 'list_types'):
            schemas.extend(module.list_types())
        else:
            # Fallback: iterate over module attributes
            for name in dir(module):
                obj = getattr(module, name)
                if isinstance(obj, type) and hasattr(obj, "__idl_typename__"):
                    try:
                        sname = schema_name(obj)
                        # Only include if schema belongs to module's package
                        if sname.startswith(f"{pkg}/"):
                            schemas.append(sname)
                    except ValueError:
                        pass  # Skip types without valid typename

    return sorted(set(schemas))  # Use set to deduplicate


def is_supported(schema: str) -> bool:
    """
    Check if a schema name is supported by this library.

    Args:
        schema: Schema name in ROS2 format (e.g., "sensor_msgs/msg/Image")

    Returns:
        True if the schema is supported, False otherwise

    Examples:
        >>> is_supported("sensor_msgs/msg/Image")
        True
        >>> is_supported("unknown_msgs/msg/Foo")
        False
    """
    parts = schema.split("/")

    if len(parts) == 3:
        package, msg_marker, type_name = parts
        if msg_marker != "msg":
            return False
    elif len(parts) == 2:
        package, type_name = parts
    else:
        return False

    if package not in _PACKAGE_MODULES:
        return False

    module = _PACKAGE_MODULES[package]

    # Use the module's is_type_supported() function if available
    if hasattr(module, 'is_type_supported'):
        return module.is_type_supported(type_name)

    # Fallback: check if attribute exists
    return hasattr(module, type_name)
