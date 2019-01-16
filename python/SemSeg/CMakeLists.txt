cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (semseg)
#find_package (Qt REQUIRED)
#find_package (VTK REQUIRED)

find_package(Boost COMPONENTS thread serialization python REQUIRED)
include_directories (${Boost_INCLUDE_DIR} /usr/include/python2.7)


find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

#set(OpenCV_DIR /usr/local/opencv320/share/OpenCV)
find_package(OpenCV REQUIRED)
#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include)
#link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv320/lib)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})



set  (project_SOURCES main.cpp semseg.cpp)
set  (project_HEADERS semseg.h)
set  (project_FORMS   semseg.ui)
#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

#INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ADD_EXECUTABLE  (main ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (main 
${QT_LIBRARIES} 
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
#${VTK_LIBRARIES}

${Boost_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libboost_python-py27.so
/usr/lib/x86_64-linux-gnu/libstdc++.so.6
/usr/lib/x86_64-linux-gnu/libpython2.7.so
)

