CC = g++
CFLAGS = -std=c++11
LDFLAGS = -lpthread

SRCS = consolewindow.cpp robot.cpp rplidar.cpp CKobuki.cpp

TARGET = consolewindow

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)


clean:
	rm -f $(TARGET)
