project(KFC_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


include_directories("/usr/include/eigen3")

add_executable(
    KFC_executed 
    main.cpp
    )


