cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-love_cloud)

#SET(VTK_DIR "/home/cbc/VTK7-build" CACHE PATH "VTK directory override" FORCE)
#find_package (VTK REQUIRED)
#find_package (Qt REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp pclviewer.cpp)
set  (project_HEADERS pclviewer.h)
set  (project_FORMS   pclviewer.ui)
#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

#INCLUDE         (${QT_USE_FILE})
#ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (qt_cloud ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (qt_cloud
# ${QT_LIBRARIES}
 ${PCL_LIBRARIES} 
#${VTK_LIBRARIES}
)

