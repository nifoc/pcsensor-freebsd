pcsensor: pcsensor.c
	${CC} ${CFLAGS} -DUNIT_TEST -o $(.TARGET) $(.ALLSRC) -lusb

clean:
	rm -f pcsensor *.o
