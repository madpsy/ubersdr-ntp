# A convenience wrapper. CMake is the build; this is for muscle memory.
BUILD ?= build

.PHONY: all clean run check install

all:
	@cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release $(if $(shell command -v ninja),-G Ninja,)
	@cmake --build $(BUILD) -j

clean:
	rm -rf $(BUILD) ubersdr-ntp_*

# Load the config and report it without connecting to anything.
check: all
	./$(BUILD)/ubersdr-ntp --config config.json --check

install: all
	@cmake --install $(BUILD)
