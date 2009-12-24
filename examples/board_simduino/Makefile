# 
# 	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>
#
#	This file is part of simavr.
#
#	simavr is free software: you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation, either version 3 of the License, or
#	(at your option) any later version.
#
#	simavr is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU General Public License for more details.
#
#	You should have received a copy of the GNU General Public License
#	along with simavr.  If not, see <http://www.gnu.org/licenses/>.

board=	simduino
firm_src = ${wildcard atmega*.c}
firmware = ${firm_src:.c=.hex}
simavr = ../../

SHELL	 = /bin/bash

IPATH = .
IPATH += ../parts
IPATH += ${simavr}/include
IPATH += ${simavr}/simavr/sim

VPATH = .
VPATH += ../parts

LDFLAGS += -lglut -lpthread

all: obj ${firmware} ${board}

include ${simavr}/Makefile.common

${board} : ${OBJ}/button.o
${board} : ${OBJ}/uart_udp.o
${board} : ${OBJ}/${board}.o
	@echo LD $@
	@gcc -MD ${CFLAGS} -o $@ $^ $(LDFLAGS) ${simavr}/simavr/libsimavr.a

clean:
	rm -rf obj *.a *.axf ${board} *.vcd