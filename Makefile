BUILD_DIR := build

.PHONY: all configure build run clean

all: build

configure:
	cmake -S . -B $(BUILD_DIR)
	ln -sf $(BUILD_DIR)/compile_commands.json compile_commands.json

build: configure
	cmake --build $(BUILD_DIR)

run: build
	./$(BUILD_DIR)/BoidSimulation

clean:
	rm -rf $(BUILD_DIR)
