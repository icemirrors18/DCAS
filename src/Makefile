IDIR =../include
CC=g++
CFLAGS=-I$(IDIR) -pthread

ODIR=obj

_DEPS = dcas.h drone.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = dcas.o drone.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

dcas: $(OBJ)
	g++ -o $@ $^ $(CFLAGS)

.PHONY: clean

clean: rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~
