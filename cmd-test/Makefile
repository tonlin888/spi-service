TARGET = cmd-test

OBJS = cmd-test.o

EXTRA_CFLAGS += -I../common

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $(CFLAGS_$@) -c -o $@ $<

install: all
	install -m 755 $(TARGET) $(ROOT_FS)/bin

clean:
	rm -f $(TARGET) *.o
