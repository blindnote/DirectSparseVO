add_library( ds_vo SHARED
    Config.cpp
    Frame.cpp
    Detector.cpp
    Memory.cpp
    BasicAlignment.cpp
    SparseAlignment.cpp
    Tracker.cpp
)

target_link_libraries( ds_vo  ${THIRD_PARTY_LIBS} )