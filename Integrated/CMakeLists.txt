cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (Integrated)
find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp integrated.cpp)
set  (project_HEADERS integrated.h)
set  (project_FORMS   integrated.ui)
set  (project_RESOURCES res.qrc)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
QT5_ADD_RESOURCES(project_RESOURCES_RCC ${project_RESOURCES})

ADD_DEFINITIONS (${QT_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ADD_EXECUTABLE  (main   ${project_SOURCES}
                        ${project_FORMS_HEADERS}
                        ${project_HEADERS_MOC}
                        ${project_RESOURCES_RCC})


TARGET_LINK_LIBRARIES (main 
${QT_LIBRARIES} 
${PCL_LIBRARIES} 
)

