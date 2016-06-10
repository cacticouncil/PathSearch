cmake_minimum_required(VERSION 3.3)

if(UNIX)
    set(CMAKE_RC_COMPILER wrc)
    set(CMAKE_RC_COMPILE_OBJECT "${CMAKE_RC_COMPILER} -I${CMAKE_CURRENT_SOURCE_DIR} -fo<OBJECT> <SOURCE>")

    set(CMAKE_CXX_LINK_EXECUTABLE "wineg++ <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
    include_directories(/usr/local/include/wine/windows)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    add_definitions(-DUSE_WINE_BUILD)
endif()

if(WIN32)
    if(MSVC)
    else()
        set(CMAKE_RC_COMPILER windres)
        set(CMAKE_RC_COMPILE_OBJECT "${CMAKE_RC_COMPILER} -I${CMAKE_CURRENT_SOURCE_DIR} <SOURCE> -o=<OBJECT>")
    endif()
#else()
endif()

enable_language(RC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(LINK_FLAGS "${LINK_FLAGS}")

project(TileLibrary)
set(TILE_SOURCE_FILES TileLibrary/Tile.cpp TileLibrary/TileMap.cpp)
add_library(TileLibrary SHARED ${TILE_SOURCE_FILES})

project(SearchLibrary)
set(SEARCH_SOURCE_FILES SearchLibrary/PathSearch.cpp)
add_library(SearchLibrary SHARED ${SEARCH_SOURCE_FILES})
target_link_libraries(SearchLibrary TileLibrary)

project(PathSearchApp.exe)
set(APP_SOURCE_FILES Application/PathSearchApp.cpp)
set(RESOURCES Resource/PathPlanner.rc ${RESOURCES})
add_executable(PathSearchApp.exe ${APP_SOURCE_FILES} ${RESOURCES})
target_link_libraries(PathSearchApp.exe SearchLibrary comctl32 gdi32 Comdlg32)

file(COPY Data DESTINATION .)
