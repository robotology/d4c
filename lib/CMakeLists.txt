# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

set(source_files src/d4c_helpers.cpp
                 src/d4c_server.cpp
                 src/d4c_client.cpp)

set(header_private_files include/iCub/d4c/private/d4c_helpers.h)

set(header_files include/iCub/d4c/d4c.h
                 include/iCub/d4c/d4c_server.h
                 include/iCub/d4c/d4c_client.h)

source_group("Source Files" FILES ${source_files})
source_group("Header Files" FILES ${header_private_files} ${header_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_library(d4c ${source_files} ${header_private_files} ${header_files})
target_link_libraries(d4c ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})

icubcontrib_export_library(d4c INTERNAL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
                               EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ctrlLib_INCLUDE_DIRS}
                               DEPENDS ctrlLib
                               DESTINATION include/iCub/d4c
                               FILES ${header_files})
