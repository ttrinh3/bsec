idf_component_register(SRCS "bsec_integration.c" "bme680.c" "main.c"
                    INCLUDE_DIRS ".")





target_link_libraries(${COMPONENT_TARGET} "-L${CMAKE_CURRENT_LIST_DIR}/lib")
target_link_libraries(${COMPONENT_TARGET} algobsec)

