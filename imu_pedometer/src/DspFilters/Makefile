CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 

$(DSPFILTER):
	$(CXX) $(CXXFLAGS) -c DspFilters/*.cpp ; mv *.o DspFilters/ 
	
	
OBJS = DspFilters/*.o 

LIBS = -lblas -llapack

$(TARGET): $(OBJS)
	$(CXX) -o  $(TARGET) $(OBJS) $(LIBS) 

all: $(DSPFILTER)

clean:
	rm -f *.o $(OBJS) $(TARGET)
