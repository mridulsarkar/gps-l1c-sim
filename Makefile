# Makefile for Linux etc.

# Detect OS and set variables
ifeq ($(OS),Windows_NT)
    EXE_EXT = .exe
    RM = del /Q /S
    MKDIR = mkdir
else
    EXE_EXT =
    RM = rm -rf
    MKDIR = mkdir -p
endif

CC = gcc
CFLAGS = -O3 -Wall -Iinclude
LDFLAGS = -lm

# Directories
SRC_DIR = src
CORE_DIR = $(SRC_DIR)/core
L1C_DIR = $(SRC_DIR)/l1c
UTILS_DIR = $(SRC_DIR)/utils
TEST_DIR = test
BIN_DIR = bin
LIB_DIR = lib
INC_DIR = include

# Source files
CORE_SRCS = $(wildcard $(CORE_DIR)/*.c)
L1C_SRCS = $(wildcard $(L1C_DIR)/*.c)
UTILS_SRCS = $(wildcard $(UTILS_DIR)/*.c)
TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)

# Object files
CORE_OBJS = $(CORE_SRCS:.c=.o)
L1C_OBJS = $(L1C_SRCS:.c=.o)
UTILS_OBJS = $(UTILS_SRCS:.c=.o)
LIB_OBJS = $(CORE_OBJS) $(L1C_OBJS) $(UTILS_OBJS)

# Targets
LIBGPSSIM = $(LIB_DIR)/libgpssim.a
MAIN_EXE = $(BIN_DIR)/gps-l1c-sim$(EXE_EXT)
TEST_EXES = $(BIN_DIR)/test_rinex$(EXE_EXT) $(BIN_DIR)/test_nav_integ$(EXE_EXT)

.PHONY: all clean dirs

all: dirs $(LIBGPSSIM) $(MAIN_EXE) $(TEST_EXES)

dirs:
	$(MKDIR) $(BIN_DIR) $(LIB_DIR)

$(LIBGPSSIM): $(LIB_OBJS)
	ar rcs $@ $^

$(MAIN_EXE): $(CORE_DIR)/gps-l1c-sim.o $(LIBGPSSIM)
	$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_DIR)/%$(EXE_EXT): $(TEST_DIR)/%.o $(LIBGPSSIM)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(BIN_DIR) $(LIB_DIR)
	$(RM) $(CORE_DIR)/*.o $(L1C_DIR)/*.o $(UTILS_DIR)/*.o $(TEST_DIR)/*.o