if (SOME_MODULE)
    message(STATUS "skipping some_module.cmake")
    message(STATUS "SOME_MODULE = " ${SOME_MODULE})
    return()
else()
    message(STATUS "running some_module.cmake")
    set(SOME_MODULE TRUE)
endif()

if (transferrable_variable)
    message(STATUS "transferrable variable in some_module in sub_directory: " ${transferrable_variable})
endif()

