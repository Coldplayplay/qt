cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (RGB_segmentation)
find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

#set  (project_SOURCES src/main.cpp src/rgbsegmentation.cpp)
#set  (project_HEADERS src/rgbsegmentation.h)
#set  (project_FORMS   src/rgbsegmentation.ui)

set  (project_SOURCES main.cpp lccpseg.cpp)
set  (project_HEADERS lccpseg.h)
set  (project_FORMS   lccpseg.ui)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

ADD_DEFINITIONS (${QT_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ADD_EXECUTABLE  (main ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (main 
${QT_LIBRARIES} 
${PCL_LIBRARIES} 
)

