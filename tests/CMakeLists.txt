add_executable(tests tests.cc)
target_link_libraries(tests PRIVATE tiny_colls)
add_test(NAME tests COMMAND tests)