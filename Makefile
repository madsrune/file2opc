CFLAGS += -std=c99 -D_BSD_SOURCE -pthread

file2opc:
	gcc $(CFLAGS) -o file2opc file2opc.c util.c
