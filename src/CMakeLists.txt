cmake_minimum_required(VERSION 2.8)
project(birds)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread -fopenmp")

find_package (Boost REQUIRED COMPONENTS filesystem)

set(SVMLIGHT_SRCS
	svmlight/svm_common.c
	svmlight/svm_hideo.c
	svmlight/svm_learn.c
	)

set(SRCS
	main.cpp
	image.cpp
	)

add_executable(birds WIN32 ${SVMLIGHT_SRCS} ${SRCS})
target_link_libraries(birds m ${Boost_FILESYSTEM_LIBRARY})
