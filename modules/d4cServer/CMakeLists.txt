# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(d4cServer)
source_group("Source Files" FILES main.cpp)
include_directories(${d4c_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(d4cServer main.cpp)
target_link_libraries(d4cServer d4c ${YARP_LIBRARIES})
install(TARGETS d4cServer DESTINATION bin)
