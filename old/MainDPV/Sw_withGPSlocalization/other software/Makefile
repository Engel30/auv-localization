all:
	g++ -pipe -O2 -Wall -W  -o client client.cpp -std=c++11
	g++ -pipe -O2 -Wall -W  -o usbl_dummy usbl_dummy.cpp -std=c++11 -lpthread
	gcc -c -Wall -Werror -fpic utilities_u/utilities_u.c
	gcc -shared -o libutilities_u.so utilities_u.o

