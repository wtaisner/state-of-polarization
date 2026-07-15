# Makefile dla libpolctrl.so + testy C
#
# Budowanie biblioteki współdzielonej z rdzeniem algorytmu.
# Wszystkie źródła w ANSI C99, bez zależności poza standardową biblioteką C.

CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -pedantic -O2 -fPIC -I$(INC_DIR)
LDFLAGS = -shared

SRC_DIR = src
INC_DIR = include
TEST_DIR = tests

# Źródła rdzenia
CORE_SRCS = $(SRC_DIR)/fixedpoint.c \
            $(SRC_DIR)/rng.c \
            $(SRC_DIR)/baseline.c \
            $(SRC_DIR)/spsa.c \
            $(SRC_DIR)/fsm.c \
            $(SRC_DIR)/bandit.c \
            $(SRC_DIR)/polctrl.c

# Wszystkie nagłówki
HEADERS = $(INC_DIR)/fixedpoint.h \
          $(INC_DIR)/polctrl.h \
          $(INC_DIR)/polctrl_internal.h \
          $(INC_DIR)/rng.h \
          $(INC_DIR)/baseline.h \
          $(INC_DIR)/spsa.h \
          $(INC_DIR)/fsm.h \
          $(INC_DIR)/bandit.h

LIB_NAME = libpolctrl.so

.PHONY: all clean test size_check

all: $(LIB_NAME)

$(LIB_NAME): $(CORE_SRCS) $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(LIB_NAME) $(CORE_SRCS)

# Pomocniczy binarny drukujący sizeof struktur (do testów parytetu)
size_check: $(TEST_DIR)/size_check.c $(HEADERS) $(CORE_SRCS)
	$(CC) $(CFLAGS) -o size_check $(TEST_DIR)/size_check.c $(CORE_SRCS) -I$(INC_DIR)

# Testy C (jeśli istnieją)
test: $(TEST_DIR)/test_fixedpoint.c $(HEADERS) $(CORE_SRCS)
	$(CC) $(CFLAGS) -o c_tests $(TEST_DIR)/test_fixedpoint.c $(CORE_SRCS) -I$(INC_DIR)
	./c_tests

clean:
	rm -f $(LIB_NAME) size_check c_tests *.o
