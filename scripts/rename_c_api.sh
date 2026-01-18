#!/bin/bash
# Script to rename C API from EdgeFirst to Ros prefix for ROS2 common interfaces
set -e
echo "C API Naming Update - Run this from repository root"
echo "Creating backups..."
cp include/edgefirst/schemas.h include/edgefirst/schemas.h.backup
cp src/ffi.rs src/ffi.rs.backup  
cp examples/c/example.c examples/c/example.c.backup
echo "Done. Run manual sed commands next."
