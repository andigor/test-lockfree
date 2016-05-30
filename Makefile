a.out: 1.cpp
	g++ -g -Os 1.cpp  -lboost_thread -lboost_system -pthread
