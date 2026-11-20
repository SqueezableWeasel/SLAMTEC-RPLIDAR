RPLIDAR_SDK ?= rplidar_sdk
SDL2_CONFIG ?= sdl2-config
CXXFLAGS ?= -O2

SDK_LIBRARY := $(RPLIDAR_SDK)/output/Linux/Release/libsl_lidar_sdk.a

.PHONY: all clean

all: rplidar_visualizer

rplidar_visualizer: rplidar_visualizer.cpp $(SDK_LIBRARY)
	$(CXX) $(CPPFLAGS) -I"$(RPLIDAR_SDK)/sdk/include" $(CXXFLAGS) -std=c++17 \
		-pthread $(LDFLAGS) -o "$@" "$<" "$(SDK_LIBRARY)" \
		$$($(SDL2_CONFIG) --cflags --libs) -lrt $(LDLIBS)

clean:
	$(RM) rplidar_visualizer
