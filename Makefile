TARGET=brobot.c

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        wait_period.c \
        wait_key.c \
	raspicam_wrapper.cpp \
        brobot.c

OBJECTS=$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

all: $(OBJECTS)
	g++ -g $(OBJECTS) -lraspicam -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -g -c $< -o $@

%.o:%.cpp
	g++ -c $< -o $@
