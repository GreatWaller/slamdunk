add_executable(slam main.cpp)
target_link_libraries(slam slamdunk ${THIRD_PARTY_LIBS})

add_executable(test test.cpp)
target_link_libraries(test ${THIRD_PARTY_LIBS})
