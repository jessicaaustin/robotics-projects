ROOT=$(PWD)
SRC=$(ROOT)/src
BIN=$(ROOT)/bin
TEST=$(ROOT)/test

CPP=g++
CPPFLAGS=-Wno-deprecated
CXXTEST=cxxtestgen.py --error-printer

## OS X 10.5
#INCLUDES=-I/opt/local/include/player-3.0 -I/opt/local/include -I/opt/local/share/player/examples/libplayerc++
#TEST_INCLUDES=-I/opt/local/share/cxxtest/ -I$(SRC)
#LIBS=-L/opt/local/lib -lplayerc++ -lboost_thread-mt -lboost_signals-mt -lplayerc -lm -lz -lplayerinterface -lplayerwkb -lplayercommon

## Ubuntu 11
INCLUDES=-I/usr/include/player-3.0 -I/usr/include -I/usr/include/player-3.0/libplayerc++ -I/usr/include/player-3.0/examples/libplayerc++ -I/usr/include/player-3.0/drivers/laser
TEST_INCLUDES=-I/usr/include/cxxtest/ -I$(SRC)
LIBS=-L/usr/lib -lplayerc++ -lboost_thread-mt -lboost_signals-mt -lplayerc -lm -lz -lplayerinterface -lplayerwkb -lplayercommon

# TODO: make test the top level target once functional tests are added
build : voronoibot test

run : voronoibot
	$(BIN)/voronoibot

test : test_runner
	$(BIN)/test_runner

voronoibot : voronoibot.o point.o
	$(CPP) $(LIBS) $(SRC)/voronoibot.o $(SRC)/point.o -o $(BIN)/voronoibot
voronoibot.o : $(SRC)/VoronoiBot.cc
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $^ -o $(SRC)/voronoibot.o
world.o : $(SRC)/World.cc
	$(CPP) $(CPPFLAGS) -c $^ -o $(SRC)/world.o
enclosure.o : $(SRC)/Enclosure.cc
	$(CPP) $(CPPFLAGS) -c $^ -o $(SRC)/enclosure.o
obstacle.o : $(SRC)/Obstacle.cc
	$(CPP) $(CPPFLAGS) -c $^ -o $(SRC)/obstacle.o
model.o : $(SRC)/Model.cc
	$(CPP) $(CPPFLAGS) -c $^ -o $(SRC)/model.o
point.o : $(SRC)/Point.cc
	$(CPP) $(CPPFLAGS) -c $^ -o $(SRC)/point.o

test_runner : test_suite.cc world.o enclosure.o obstacle.o model.o point.o
	$(CPP) $(CPPFLAGS) $(TEST_INCLUDES) -o $(BIN)/test_runner $(TEST)/test_suite.cc $(SRC)/world.o $(SRC)/enclosure.o $(SRC)/obstacle.o $(SRC)/model.o $(SRC)/point.o
test_suite.cc : $(TEST)/*Test.hh
	$(CXXTEST) -o $(TEST)/test_suite.cc $^

clean :
	rm $(BIN)/* $(SRC)/*.o $(TEST)/test_suite.cc
