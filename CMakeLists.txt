
add_executable(body_tracking main.c)


# Dependencies of this library
target_link_libraries(simple_sample PRIVATE 
    k4a
    k4abt
    )


