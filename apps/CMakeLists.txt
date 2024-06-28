include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external")

add_subdirectory(roofer-app)
add_subdirectory(crop)
add_subdirectory(reconstruct)

if(WIN32)
    install (DIRECTORY ${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/share/proj
            DESTINATION "share"
            FILES_MATCHING PATTERN "*" PATTERN "*.cmake" EXCLUDE)
    install(DIRECTORY ${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/share/gdal
            DESTINATION "share"
            FILES_MATCHING PATTERN "*" PATTERN "*.cmake" EXCLUDE)
endif()

# add polyscope
# if(${RF_BUILD_GUI})
#   add_subdirectory(external/polyscope)

#   add_executable(ransac_demo ransac_demo.cpp)
#   set_target_properties(
#     ransac_demo PROPERTIES
#     CXX_STANDARD 20
#   )
#   target_link_libraries(ransac_demo PRIVATE roofer polyscope cmake_git_version_tracking)
# endif()
