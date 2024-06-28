set(APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/roofer-app.cpp")

add_executable("roofer" ${APP_SOURCES})
set_target_properties("roofer" PROPERTIES CXX_STANDARD 20)
target_link_libraries("roofer" PRIVATE roofer-extra cmake_git_version_tracking
                                       fmt::fmt libfork::libfork)

install(
  TARGETS "roofer"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
