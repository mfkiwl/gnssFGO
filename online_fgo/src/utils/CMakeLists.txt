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

set(ONLINEFGO_UTIL_NAME
    ${ONLINEFGO_PREFIX}_util
)
add_library(${ONLINEFGO_UTIL_NAME} 
    SHARED
    GPUtils.cpp
    Pose3utils.cpp
)
target_include_directories(${ONLINEFGO_UTIL_NAME}
    PUBLIC
    ${ONLINEFGO_INCLUDE}
)
target_link_libraries(${ONLINEFGO_UTIL_NAME}
    PUBLIC
    ${ONLINEFGO_LINK}
)
ament_target_dependencies(${ONLINEFGO_UTIL_NAME}
    PUBLIC
    ${ONLINEFGO_ROS_DEP}
)
set(ONLINEFGO_BUILD_TARGET
    ${ONLINEFGO_BUILD_TARGET}
    ${ONLINEFGO_UTIL_NAME}
    PARENT_SCOPE
)
