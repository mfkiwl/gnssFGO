#  Copyright 2024 Institute of Automatic Control RWTH Aachen University
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
#          Jiandong Chen (jiandong.chen@rwth-aachen.de)

# **************************************************************************** #
# ************************** helper ************************************* #
macro(onlinefgo_node_helper node_name src_file_name)
    add_executable(${node_name}
        "${src_file_name}"
    )
    target_include_directories(${node_name}
        PUBLIC
        ${ONLINEFGO_INCLUDE}
    )
    target_link_libraries(${node_name}
        PUBLIC
        ${ONLINEFGO_LINK}
        ${ONLINEFGO_BUILD_TARGET}
    )
    ament_target_dependencies(${node_name}
        PUBLIC
        ${node_dep_ros}
    )
    install(
        TARGETS ${node_name}
        DESTINATION lib/${PROJECT_NAME}  # ros2 needs all executables in the lib directory
    )
endmacro()

# **************************************************************************** #
# ************************** boreas_node ************************************* #
# set(boreas_node_src_file
#     nodeOnlineFGOBoreas.cpp 
#     nodeOfflineFGOTimeCentric.cpp
# )
# onlinefgo_node_helper(boreas_node ${boreas_node_src_file})

# ************************** learning_gp_node ******************************** #
onlinefgo_node_helper(learning_gp_node nodeOfflineLearningGP.cpp)

# ************************** online_fgo_node ********************************* #
onlinefgo_node_helper(online_fgo_node nodeOnlineFGOTimeCentric.cpp)

# ************************** urbanloco_node ********************************** #
onlinefgo_node_helper(urbanloco_node nodeOnlineFGOTimeCentricUrbanLoco.cpp)

# ************************** gt_node ***************************************** #
onlinefgo_node_helper(gt_node nodeOnlineFGOGT.cpp)
