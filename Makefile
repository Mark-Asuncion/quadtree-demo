CPP =g++
CPPLDFLAGS=-L$(shell pwd)/raylib-5.0_linux_amd64/lib -lraylib -I$(shell pwd)/raylib-5.0_linux_amd64/include/
CPPFLAGS =-Wextra -Wall -pedantic -std=c++17

SOURCE = src/main.cpp
OUT = build/quadtree-demo

.PHONY: compile_flags.txt

$(OUT): $(SOURCE)
	$(CPP) $(CPPFLAGS) -o $(OUT) $(SOURCE) $(CPPLDFLAGS)

compile_flags.txt:
	echo "$(CPPLDFLAGS) $(CPPFLAGS)" | tr ' ' '\n' > compile_flags.txt; \
