CC      := gcc
CFLAGS  := -O2
TARGET  := example
SRCS    := example.c pipwm.c pipwm.h

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)

.PHONY: all clean
