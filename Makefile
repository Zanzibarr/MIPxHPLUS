CC = g++
SRCDIR = code/src
INCDIR = code/include
BUILDIR = code/build

SRCS = $(wildcard $(SRCDIR)/*.cpp)
OBJS = $(SRCS:$(SRCDIR)/%.cpp=$(BUILDIR)/%.o)
DEPS = $(wildcard $(INCDIR)/*.hpp)
TARGET = $(BUILDIR)/main

VERBOSE = 10

.PHONY: all clean

all: $(TARGET)

$(TARGET) : $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) -O3 && make clean

debug: $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) -g && make clean

$(BUILDIR)/%.o : $(SRCDIR)/%.cpp $(DEPS)
	$(CC) -c $< -o $@ -D VERBOSE=$(VERBOSE)

clean:
	rm -f $(OBJS)

clear:
	rm -f $(OBJS) $(TARGET)
