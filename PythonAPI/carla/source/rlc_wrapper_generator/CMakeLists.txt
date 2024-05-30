project(rlc-generator)
cmake_minimum_required(VERSION 3.22)
find_package(LLVM REQUIRED)
find_package(Clang REQUIRED)
add_executable(generator ./gen_rlc_wrapper.cpp)
target_include_directories(generator PUBLIC ${LLVM_INCLUDE_DIRS})
target_compile_definitions(generator PUBLIC ${LLVM_DEFINITIONS_LIST})
target_compile_options(generator PUBLIC -fno-rtti)
target_link_libraries(generator PUBLIC    clangFrontend
  clangSerialization
  clangDriver
  clangParse
  clangSema
  clangAnalysis
  clangAST
  clangBasic
  clangEdit
  clangLex
  clangTooling)
