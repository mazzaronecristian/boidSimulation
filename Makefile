BUILD_DIR := build

.PHONY: all configure build run run-benchmark clean

all: build

configure:
	cmake -S . -B $(BUILD_DIR)
	ln -sf $(BUILD_DIR)/compile_commands.json compile_commands.json

build: configure
	cmake --build $(BUILD_DIR)

run: build
	./$(BUILD_DIR)/BoidSimulation

run-benchmark: build
	./benchmark $(MODE) $(N)

clean:
	rm -rf $(BUILD_DIR)
