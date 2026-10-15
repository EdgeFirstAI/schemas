# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

# Message for representing information about a machine learning model
# Typically used to communicate details about a model over ROS

# Element type codes for input_dtype / output_dtype. These are the EdgeFirst
# HAL tensor dtype codes (edgefirst_tensor_abi::EfDtype, EF_DTYPE_* in
# edgefirst/tensor.h) under the same names. HAL has no code for an unknown or
# opaque element type, so DTYPE_UNKNOWN sits outside the HAL range.
uint8 DTYPE_U8      = 0
uint8 DTYPE_I8      = 1
uint8 DTYPE_U16     = 2
uint8 DTYPE_I16     = 3
uint8 DTYPE_U32     = 4
uint8 DTYPE_I32     = 5
uint8 DTYPE_U64     = 6
uint8 DTYPE_I64     = 7
uint8 DTYPE_F16     = 8
uint8 DTYPE_F32     = 9
uint8 DTYPE_F64     = 10
uint8 DTYPE_UNKNOWN = 255

# Deprecated: codes for the legacy input_type / output_type fields. They
# number the same element types differently from HAL; use DTYPE_* with
# input_dtype / output_dtype instead.
uint8 RAW     = 0
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 FLOAT16 = 5
uint8 INT32   = 6
uint8 UINT32  = 7
uint8 FLOAT32 = 8
uint8 INT64   = 9
uint8 UINT64  = 10
uint8 FLOAT64 = 11
uint8 STRING  = 12

std_msgs/Header header          # Metadata including timestamp and coordinate frame
uint32[] input_shape     # Shape of the input tensor(s) in the format "height,width,channels" or "height,width,depth,channels"
uint8 input_type      # Deprecated: input element type as a legacy code (RAW ... STRING); mirrors input_dtype
uint32[] output_shape    # Shape of the output tensor(s) in the format "height,width,channels" or "height,width,depth,channels"
uint8 output_type     # Deprecated: output element type as a legacy code (RAW ... STRING); mirrors output_dtype
string[] labels        # Array of strings representing the labels used by the model, empty if no labels available
string model_types     # Model tasks/types (e.g., ["object_detection", "classification"])
string model_format    # Format of the model (e.g., "DeepViewRT", "HailoRT", "RKNN", "TensorRT", "TFLite")
string model_name      # Name of the model (if available), otherwise use filename without extension or path
uint8 input_dtype     # Input element type as a DTYPE_* code; absent from messages recorded before it was added
uint8 output_dtype    # Output element type as a DTYPE_* code; absent from messages recorded before it was added
