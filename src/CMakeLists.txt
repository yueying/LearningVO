# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件
INCLUDE_DIRECTORIES(  ${OpenCV_INCLUDE_DIRS}  )

file(GLOB lib_vo ../include/*.h *.cpp )
ADD_EXECUTABLE( vo ${lib_vo} )
TARGET_LINK_LIBRARIES( vo ${OpenCV_LIBS}  )