cmake_minimum_required(VERSION 2.8.3)
project (svm_training)

find_package(PkgConfig)
pkg_check_modules(GAMLLIBSVM gaml-libsvm REQUIRED)

add_executable(train_svm train_svm.cpp)
include_directories(${GAMLLIBSVM_INCLUDE_DIRS})
target_link_libraries     (train_svm "-lsvm -lm")
target_include_directories(train_svm PUBLIC ${GAMLLIBSVM_INCLUDE_DIRS})
target_compile_options    (train_svm PUBLIC --std=c++11 ${GAMLLIBSVM_CFLAGS_OTHER})

#  g++ -o train_svm -Wall -ansi -O3 train_svm.cpp `pkg-config --libs --cflags gaml-libsvm`
