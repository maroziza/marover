CFLAGS=-DJS_SHARED_LIBRARY
%.so: %.o
	$(CC) $(LDFLAGS) -shared -o $@ $^

ioctl.o: ioctl.c
ioctl.so: ioctl.o
all: ioctl.so

clean:
	rm ioctl.so ioctl.o