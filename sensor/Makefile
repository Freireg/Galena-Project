TARGET=max30102_i2c
BUILD_DIR := build

ifeq (,$(TOOLCHAIN_PREFIX))
$(error TOOLCHAIN_PREFIX is not set)
endif

ifeq (,$(CFLAGS))
$(error CFLAGS is not set)
endif

ifeq (,$(LDFLAGS))
$(error LDFLAGS is not set)
endif

# Default rule
all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET)

# Create build dir if not exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

CC = $(TOOLCHAIN_PREFIX)gcc

SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(SOURCE))

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: all clean
clean:
	rm -rf $(BUILD_DIR)

