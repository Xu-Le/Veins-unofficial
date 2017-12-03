# This file is part of a max throughput algorithm of STAG
CC = g++
CXXFLAGS = -Wall -g -std=c++11
STAG: Log.o Utils.o STAG.o main.o
	$(CC) -o $@ $^
Log.o: Log.cpp Log.h
	$(CC) $(CXXFLAGS) -c $<
Utils.o: Utils.cpp Utils.h Timer.h
	$(CC) $(CXXFLAGS) -c $<
STAG.o: STAG.cpp STAG.h Utils.h
	$(CC) $(CXXFLAGS) -c $<
main.o: main.cpp STAG.h Utils.h Timer.h
	$(CC) $(CXXFLAGS) -c $<
.PHONY: clean
clean:
	rm -f *.h~
	rm -f *.cpp~
	rm -f *.o
	rm -f STAG