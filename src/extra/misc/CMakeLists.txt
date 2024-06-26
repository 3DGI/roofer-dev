set(LIBRARY_SOURCES
    "NodataCircleComputer.cpp"
    "PC2MeshDistCalculator.cpp"
    "PointcloudRasteriser.cpp"
    "projHelper.cpp"
    "select_pointcloud.cpp"
    "Vector2DOpsGEOS.cpp")
if(RF_USE_VAL3DITY)
    list(APPEND LIBRARY_SOURCES "Val3dator.cpp")
endif()
set(LIBRARY_HEADERS
    "${ROOFER_INCLUDE_DIR}/roofer/misc/NodataCircleComputer.hpp"
    "${ROOFER_INCLUDE_DIR}/roofer/misc/PC2MeshDistCalculator.hpp"
    "${ROOFER_INCLUDE_DIR}/roofer/misc/PointcloudRasteriser.hpp"
    "${ROOFER_INCLUDE_DIR}/roofer/misc/projHelper.hpp"
    "${ROOFER_INCLUDE_DIR}/roofer/misc/select_pointcloud.hpp"
    "${ROOFER_INCLUDE_DIR}/roofer/misc/Vector2DOps.hpp")
if(RF_USE_VAL3DITY)
    list(APPEND LIBRARY_HEADERS "${ROOFER_INCLUDE_DIR}/roofer/misc/Val3dator.hpp")
endif()
set(LIBRARY_INCLUDES
    "${ROOFER_INCLUDE_DIR}")

add_library("misc" OBJECT ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_include_directories("misc" PUBLIC ${LIBRARY_INCLUDES})

set(MISC_LINK_LIBS
    CGAL::CGAL
    CGAL::CGAL_Core # Eigen3::Eigen
    GEOS::geos_c
    PROJ::proj
)
if(RF_USE_VAL3DITY)
    list(APPEND MISC_LINK_LIBS val3dity)
endif()

target_link_libraries("misc" PUBLIC ${MISC_LINK_LIBS})
