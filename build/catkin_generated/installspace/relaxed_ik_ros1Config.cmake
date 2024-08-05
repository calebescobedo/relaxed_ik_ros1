# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(relaxed_ik_ros1_CONFIG_INCLUDED)
  return()
endif()
set(relaxed_ik_ros1_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(relaxed_ik_ros1_SOURCE_PREFIX /home/caleb/catkin_ws/src/relaxed_ik_ros1)
  set(relaxed_ik_ros1_DEVEL_PREFIX /home/caleb/catkin_ws/src/relaxed_ik_ros1/build/devel)
  set(relaxed_ik_ros1_INSTALL_PREFIX "")
  set(relaxed_ik_ros1_PREFIX ${relaxed_ik_ros1_DEVEL_PREFIX})
else()
  set(relaxed_ik_ros1_SOURCE_PREFIX "")
  set(relaxed_ik_ros1_DEVEL_PREFIX "")
  set(relaxed_ik_ros1_INSTALL_PREFIX /usr/local)
  set(relaxed_ik_ros1_PREFIX ${relaxed_ik_ros1_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'relaxed_ik_ros1' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(relaxed_ik_ros1_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include " STREQUAL " ")
  set(relaxed_ik_ros1_INCLUDE_DIRS "")
  set(_include_dirs "include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'yepw <yepingw@outlook.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${relaxed_ik_ros1_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'relaxed_ik_ros1' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'relaxed_ik_ros1' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
    endif()
    _list_append_unique(relaxed_ik_ros1_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND relaxed_ik_ros1_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND relaxed_ik_ros1_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT relaxed_ik_ros1_NUM_DUMMY_TARGETS)
      set(relaxed_ik_ros1_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::relaxed_ik_ros1::wrapped-linker-option${relaxed_ik_ros1_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR relaxed_ik_ros1_NUM_DUMMY_TARGETS "${relaxed_ik_ros1_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::relaxed_ik_ros1::wrapped-linker-option${relaxed_ik_ros1_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND relaxed_ik_ros1_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND relaxed_ik_ros1_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND relaxed_ik_ros1_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /usr/local/lib;/home/caleb/catkin_ws/devel/lib;/home/caleb/catkin_ws/devel_isolated/franka_lock_unlock/lib;/home/caleb/catkin_ws/devel_isolated/pilz_industrial_motion_planner/lib;/home/caleb/catkin_ws/devel_isolated/pilz_industrial_motion_planner_testutils/lib;/home/caleb/catkin_ws/devel_isolated/moveit_tutorials/lib;/home/caleb/catkin_ws/devel_isolated/moveit_calibration_gui/lib;/home/caleb/catkin_ws/devel_isolated/moveit_visual_tools/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_control_interface/lib;/home/caleb/catkin_ws/devel_isolated/moveit_simple_controller_manager/lib;/home/caleb/catkin_ws/devel_isolated/moveit_setup_assistant/lib;/home/caleb/catkin_ws/devel_isolated/moveit_servo/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_visualization/lib;/home/caleb/catkin_ws/devel_isolated/moveit_planners_chomp/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_planning_interface/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_benchmarks/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_warehouse/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_robot_interaction/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_perception/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_manipulation/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_move_group/lib;/home/caleb/catkin_ws/devel_isolated/moveit_planners_ompl/lib;/home/caleb/catkin_ws/devel_isolated/moveit_kinematics/lib;/home/caleb/catkin_ws/devel_isolated/moveit_fake_controller_manager/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_planning/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros_occupancy_map_monitor/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_prbt_ikfast_manipulator_plugin/lib;/home/caleb/catkin_ws/devel_isolated/moveit_chomp_optimizer_adapter/lib;/home/caleb/catkin_ws/devel_isolated/chomp_motion_planner/lib;/home/caleb/catkin_ws/devel_isolated/moveit_core/lib;/home/caleb/catkin_ws/devel_isolated/srdfdom/lib;/home/caleb/catkin_ws/devel_isolated/rviz_visual_tools/lib;/home/caleb/catkin_ws/devel_isolated/relaxed_ik_ros1/lib;/home/caleb/catkin_ws/devel_isolated/panda_moveit_config/lib;/home/caleb/catkin_ws/devel_isolated/moveit_runtime/lib;/home/caleb/catkin_ws/devel_isolated/moveit_ros/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_prbt_support/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_prbt_pg70_support/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_prbt_moveit_config/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_pr2_description/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_panda_moveit_config/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_panda_description/lib;/home/caleb/catkin_ws/devel_isolated/moveit_commander/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_fanuc_moveit_config/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_fanuc_description/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources_dual_panda_moveit_config/lib;/home/caleb/catkin_ws/devel_isolated/moveit_resources/lib;/home/caleb/catkin_ws/devel_isolated/moveit_plugins/lib;/home/caleb/catkin_ws/devel_isolated/moveit_planners/lib;/home/caleb/catkin_ws/devel_isolated/moveit_msgs/lib;/home/caleb/catkin_ws/devel_isolated/moveit_calibration_plugins/lib;/home/caleb/catkin_ws/devel_isolated/moveit/lib;/home/caleb/catkin_ws/devel_isolated/geometric_shapes/lib;/home/caleb/catkin_ws/devel_isolated/franka_visualization/lib;/home/caleb/catkin_ws/devel_isolated/franka_ros/lib;/home/caleb/catkin_ws/devel_isolated/franka_gazebo/lib;/home/caleb/catkin_ws/devel_isolated/franka_example_controllers/lib;/home/caleb/catkin_ws/devel_isolated/franka_control/lib;/home/caleb/catkin_ws/devel_isolated/franka_hw/lib;/home/caleb/catkin_ws/devel_isolated/franka_msgs/lib;/home/caleb/catkin_ws/devel_isolated/franka_gripper/lib;/home/caleb/catkin_ws/devel_isolated/franka_description/lib;/home/caleb/catkin_ws/devel_isolated/depthai_ros_driver/lib;/home/caleb/catkin_ws/devel_isolated/depthai_examples/lib;/home/caleb/catkin_ws/devel_isolated/depthai_bridge/lib;/home/caleb/catkin_ws/devel_isolated/depthai_ros_msgs/lib;/home/caleb/catkin_ws/devel_isolated/depthai_filters/lib;/home/caleb/catkin_ws/devel_isolated/depthai_descriptions/lib;/home/caleb/catkin_ws/devel_isolated/depthai-ros/lib;/home/caleb/catkin_ws/devel_isolated/baldor/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(relaxed_ik_ros1_LIBRARY_DIRS ${lib_path})
      list(APPEND relaxed_ik_ros1_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'relaxed_ik_ros1'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND relaxed_ik_ros1_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(relaxed_ik_ros1_EXPORTED_TARGETS "relaxed_ik_ros1_generate_messages_cpp;relaxed_ik_ros1_generate_messages_eus;relaxed_ik_ros1_generate_messages_lisp;relaxed_ik_ros1_generate_messages_nodejs;relaxed_ik_ros1_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${relaxed_ik_ros1_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 relaxed_ik_ros1_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${relaxed_ik_ros1_dep}_FOUND)
      find_package(${relaxed_ik_ros1_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${relaxed_ik_ros1_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(relaxed_ik_ros1_INCLUDE_DIRS ${${relaxed_ik_ros1_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(relaxed_ik_ros1_LIBRARIES ${relaxed_ik_ros1_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${relaxed_ik_ros1_dep}_LIBRARIES})
  _list_append_deduplicate(relaxed_ik_ros1_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(relaxed_ik_ros1_LIBRARIES ${relaxed_ik_ros1_LIBRARIES})

  _list_append_unique(relaxed_ik_ros1_LIBRARY_DIRS ${${relaxed_ik_ros1_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(relaxed_ik_ros1_EXPORTED_TARGETS ${${relaxed_ik_ros1_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "relaxed_ik_ros1-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${relaxed_ik_ros1_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
