# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# octomap
FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )


ADD_LIBRARY( detect_objects detect_Objects.cpp )
TARGET_LINK_LIBRARIES( detect_objects
        ${OpenCV_LIBS})

ADD_LIBRARY( read_file octoMap_processing.cpp)
TARGET_LINK_LIBRARIES( read_file
        ${OCTOMAP_LIBRARIES}
        ${OpenCV_LIBS})


ADD_LIBRARY( car_space_norm car_space_norm.cpp )
TARGET_LINK_LIBRARIES( car_space_norm
        ${OpenCV_LIBS})

ADD_LIBRARY( ransac_line_detection line_ransac.cpp )
TARGET_LINK_LIBRARIES( ransac_line_detection
        ${OpenCV_LIBS})


ADD_LIBRARY(read_folder readFolder.cpp)
TARGET_LINK_LIBRARIES( read_folder)

ADD_EXECUTABLE( detect main.cpp )
TARGET_LINK_LIBRARIES( detect
        read_folder
        read_file
        detect_objects
        car_space_norm
        ransac_line_detection
        ${OCTOMAP_LIBRARIES}
        ${OpenCV_LIBS})
