CC = gcc
CFLAGS = -g --std=c99 -Wall
RM = rm

TARGET = interpreter

all: $(TARGET)

$(TARGET): $(TARGET).c
				$(CC) $(CLFAGS) $(TARGET).c -o $(TARGET)

clean:
		$(RM) $(TARGET)
