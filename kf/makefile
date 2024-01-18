# Compiler and flags
CXX = clang++
CXXFLAGS = -std=c++20 -Wall

# Directories
SRC_DIR = src
INCLUDE_DIR = include
BIN_DIR = bin
EIGEN_DIR = /usr/include/eigen3

# Source files and objects
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(BIN_DIR)/%.o, $(SRC_FILES))

# Executable
TARGET = $(BIN_DIR)/run_kf_test.out

# Build rules
all: $(TARGET)

$(TARGET): $(OBJ_FILES)
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -I$(EIGEN_DIR) $^ -o $@

$(BIN_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -I$(EIGEN_DIR) -c $< -o $@

# Clean rule
clean:
	rm -f $(BIN_DIR)/*.o $(TARGET)