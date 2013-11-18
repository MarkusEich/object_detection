rock_vizkit_plugin(object_detection-viz
    PrimitiveObjectVisualization.cpp
    DEPS object_detection
    DEPS_PKGCONFIG base-viz
    MOC PrimitiveObjectVisualization.hpp
    HEADERS PrimitiveObjectVisualization.hpp)

