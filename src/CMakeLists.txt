cmake_minimum_required(VERSION 2.8)
project(birds)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")

find_package (Boost REQUIRED)
find_package (Qt4 REQUIRED)

set (CMAKE_AUTOMOC TRUE)

set (QT_USE_QTOPENGL TRUE)
include (${QT_USE_FILE})
set(SVMLIGHT_SRCS
	svmlight/svm_common.c
	svmlight/svm_hideo.c
	svmlight/svm_learn.c
	)

set(SRCS
	main.cpp
	glwidget.cpp
	mainwindow.cpp
	)

add_executable(birds WIN32 ${SVMLIGHT_SRCS} ${SRCS})
target_link_libraries(birds m ${QT_LIBRARIES})
