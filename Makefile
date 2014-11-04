CC = g++
CF_FLAGS = -Wall -g
LIB_FLAGS = -std=gnu++0x
RM = rm -fr
MKDIR = mkdir -p

all: easyBlack memGPIO-LEDs memGPIO-LED-w-Button

memGPIO.o:
	$(CC) $(CF_FLAGS) $(LIB_FLAGS) -c ./src/memGPIO.cpp -o ./src/memGPIO.o

utils.o:
	$(CC) $(CF_FLAGS) $(LIB_FLAGS) -c ./src/utils.cpp -o ./src/utils.o

libeasyBlack.a: memGPIO.o utils.o
	$(MKDIR) ./lib
	ar -rc ./lib/libeasyBlack.a ./src/memGPIO.o ./src/utils.o
	ranlib ./lib/libeasyBlack.a

easyBlack: libeasyBlack.a

memGPIO-LEDs: easyBlack
	$(CC) $(CF_FLAGS) -L./lib ./examples/memGPIO-LEDs.cpp -leasyBlack -o ./examples/memGPIO-LEDs

memGPIO-LED-w-Button: easyBlack
	$(CC) $(CF_FLAGS) -L./lib ./examples/memGPIO-LED-w-Button.cpp -leasyBlack -o ./examples/memGPIO-LED-w-Button

examples: memGPIO-LEDs memGPIO-LED-w-Button

clean:
	$(RM) ./src/memGPIO.o ./src/utils.o ./lib ./examples/memGPIO-LEDs ./examples/memGPIO-LED-w-Button

objclean:
	$(RM) ./src/memGPIO.o ./src/utils.o

libclean:
	$(RM) ./src/memGPIO.o ./src/utils.o ./lib

exclean:
	$(RM) ./examples/memGPIO-LEDs ./examples/memGPIO-LED-w-Button

re: clean all
