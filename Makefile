# Makefile for myactuator_rmd project

# Setting the default directory
BUILD_DIR = build
CMAKE = cmake

# Default target is all
.PHONY: all clean rebuild config

# Build target (configure + build)
all: config
	@echo "Build execution in progress..."
	@$(CMAKE) --build $(BUILD_DIR) -- -j$$(nproc)
	@echo "Build completed!"

# CMake configuration
config: $(BUILD_DIR)
	@echo "CMake configuration in progress..."
	@cd $(BUILD_DIR) && $(CMAKE) .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@echo "CMake configuration completed!"

# Build directory creation
$(BUILD_DIR):
	@echo "Build directory creation in progress..."
	@mkdir -p $(BUILD_DIR)
	@echo "Build directory creation completed!"

# Clean (build directory deletion)
clean:
	@echo "Build directory deletion in progress..."
	@rm -rf $(BUILD_DIR)
	@echo "Build directory deletion completed!"

# Rebuild (clean + all)
rebuild: clean all

# Run v161_example (after build)
run: all
	@echo "v161_example execution in progress..."
	@./$(BUILD_DIR)/v161_example
	@echo "v161_example execution completed!"

# Help target
help:
	@echo "Available targets:"
	@echo "  all      : Project build (default target)"
	@echo "  config   : CMake configuration only"
	@echo "  clean    : Build directory deletion"
	@echo "  rebuild  : Clean then full rebuild"
	@echo "  run      : Project build then main_app execution"
	@echo "  help     : Display this help message"
