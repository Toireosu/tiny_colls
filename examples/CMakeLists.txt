FetchContent_Declare(
    raylib    
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)

FetchContent_MakeAvailable(raylib)

add_executable(Example example.cc)
target_link_libraries(Example PRIVATE tiny_colls raylib)

add_executable(Demo demo.cc)
target_link_libraries(Demo PRIVATE tiny_colls raylib)