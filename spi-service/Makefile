CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -pthread

SRCS := main.cpp IpcManager.cpp SpiManager.cpp
OBJS := $(SRCS:.cpp=.o)

spi-service: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) spi-service
