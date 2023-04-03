#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  add_rosrs_setup_env HECTOR_USE_WHOLE_BODY_PLANNER "ON=1,OFF=0" "Additionally start the whole-body planner for optional path execution."
  add_rosrs_setup_env HECTOR_USE_WHOLE_BODY_PLANNER_AS_DEFAULT "ON=1,OFF=0" "Use the whole-body planner for default path execution."
fi
