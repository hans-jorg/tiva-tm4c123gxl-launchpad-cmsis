####
##  @file     Makefile
##  @brief    General makefile for TM4C123
##  @version  V1.0
##  @date     23/01/2016
##
##  @note     CMSIS library used
##  @note     options
##  @param all      generate image file
##  @param flash    write image file into microcontroller
##  @param edit     open source files for editing
##  @param debug    starts a debug section
##  @param openocd  start opencd for debugging
##  @param size     show code size
##  @param nm       list symbols
##  @param dump     show object code in assembly format
##
##

###############################################################################
# Main parameters                                                             #
###############################################################################
#
# Program name
#
PROGNAME=blink

#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM
# Used to define part in header file
PARTCLASS=TM4C123GH6PM
# Used to find correct CMSIS include file
PARTCLASSCMSIS=TM4C123

#
# Source files
#
SRCFILES=$(wildcard *.c)
#SRCFILES= main.c

#
# Include the common make definitions.
#
PREFIX:=arm-none-eabi

#
# CMSIS Dir
#
CMSISDIR=../../CMSIS

#
# Additional libraries/headers
#
#

#
# Main target
#
default: all

#
# Verbose output for Make (Coment to have verbose output)
#
MAKEFLAGS+= --silent

###############################################################################
# Generally it is not needed to modify the following lines                    #
###############################################################################

#
# Folder for object files
#
OBJDIR=gcc

#
# Object files
#
OBJFILES=$(addprefix ${OBJDIR}/,$(SRCFILES:.c=.o))

# Set the compiler CPU/FPU options.
#
CPUFLAGS=-mcpu=cortex-m4
FPUFLAGS=-mfpu=fpv4-sp-d16 -mfloat-abi=softfp

#
# Include path
#
CMSISDEVINCDIR=$(CMSISDIR)/Device/TI/TM4C/Include
CMSISINCDIR=${CMSISDIR}/Include
INCLUDEPATH=${CMSISDEVINCDIR} ${CMSISINCDIR}

###############################################################################
# Compilation parameters                                                      #
###############################################################################

#
#
# The flags passed to the assembler.
#
AFLAGS=-mthumb \
       ${CPUFLAGS}  \
       ${FPUFLAGS}  \
       $(addprefix -I ,${INCLUDEPATH})           \

#
# The flags passed to the compiler.
#
CFLAGS=-mthumb             \
       ${CPUFLAGS}              \
       ${FPUFLAGS}              \
       $(addprefix -I ,${INCLUDEPATH})           \
       -D${PARTCLASS}            \
       -ffunction-sections \
       -fdata-sections     \
       -std=c11            \
       -pedantic           \
       -DPART_${PART}

###############################################################################
#   Commands                                                                  #
###############################################################################

#
# Terminal application (used to open new windows in debug)
#
#TERMAPP=xterm
TERMAPP=gnome-terminal

#
# The command to flash the device
#
FLASHER=lm4flash
#FLASHER=openocd

#
# The command for calling the compiler.
#
CC=${PREFIX}-gcc

#
# The command for calling the library archiver.
#
AR=${PREFIX}-ar

#
# The command for calling the linker.
#
LD=${PREFIX}-ld

#
# The command for extracting images from the linked executables.
#
OBJCOPY=${PREFIX}-objcopy

#
# The command for disassembly
#
OBJDUMP=${PREFIX}-objdump

#
# Flags for disassembler
#
ODFLAGS=-S -D

#
# The command for listing size of code
#
OBJSIZE=${PREFIX}-size

#
# The command for listing symbol table
#
OBJNM=${PREFIX}-nm

#
# The flags passed to the linker.
#
LDFLAGS=--gc-sections

#
# Debugger
#
GDB=$(PREFIX)-gdb

#
# GUI Editor
#
GUIEDITOR=gedit

###############################################################################
#   Command parameters                                                        #
###############################################################################

#
# Serial terminal communication
#
TTYTERM=/dev/ttyACM0
TTYBAUD=155200

#
# Serial terminal emulator
#
# Use one of configuration below
# cu
#TTYPROG=cu
#TTYPARMS=-l $(TTYTERM) -s $(TTYBAUD)
# screen
#TTYPROG=screen
#TTYPARMS= $(TTYTERM) $(TTYBAUD)
# minicom
TTYPROG=minicom
TTYPARMS=-D $(TTYTERM) -b $(TTYBAUD)
# putty
#TTYPROG=putty
# tip
#TTYPROG=tip
#TTYPARMS=-$(TTYBAUD) $(TTYTERM)

#
# Parameters for OpenOCD
#
OPENOCDBOARD=/usr/share/openocd/scripts/board/stm32l4discovery.cfg
OPENOCDFLASHSCRIPT=$(OBJDIR)/flash.ocd

#
# The flags passed to the debugger
#
GDBFLAGS=-x $(GDBINIT) -n

#
# GDB initialization file
#
GDBINIT=$(OBJDIR)/gdbinit

#
# Get the location of libgcc.a from the GCC front-end.
#
LIBGCC:=${shell ${CC} ${CFLAGS} -print-libgcc-file-name}

#
# Get the location of libc.a from the GCC front-end.
#
LIBC:=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}

#
# Get the location of libm.a from the GCC front-end.
#
LIBM:=${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

#
# Generate debug version if DEBUG is set
#
ifneq ($DEBUG,)
CFLAGS+=-g -D DEBUG -O0
else
CFLAGS+=-Os
endif

#
# Additional Flags
#
CFLAGS+= -Wuninitialized -Werror

#
# Controlling dependencies on header files
#
DEPFLAGS=-MT $@  -MMD -MP -MF ${OBJDIR}/$*.d

#
# Linker script
#
#LINKERSCRIPT=$(PROGNAME).ld
LINKERSCRIPT=tm4c123.ld
#LINKERSCRIPT=$(shell echo $(PART)| tr A-Z a-z).ld

#
# Entry Point
#
ENTRY=Reset_Handler

#
# Linker flags
#
LFLAGS=-gc-sections

###############################################################################
#   Rules                                                                     #
###############################################################################

#
# The rule for building the object file from each C source file.
#
${OBJDIR}/%.o: %.c
	@echo "  Compiling           ${<}";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}

#
# The rule for building the object file from each assembly source file.
#
${OBJDIR}/%.o: %.S
	@echo "  Assembling          ${<}";
	${CC} -c ${AFLAGS} -D${OBJDIR} -o ${@} -c ${<}

#
# The rule for creating an object library.
#
${OBJDIR}/%.a:
	@echo "  Archiving           ${@}";
	${AR} -cr ${@} ${^}

#
# The rule for linking the application.
#
${OBJDIR}/%.axf:  $(OBJFILES)
	@echo "  Linking             ${@} ";
	${LD} -T '${LINKERSCRIPT}' --entry '${ENTRY}' \
	      ${LDFLAGS} -o ${@} $(OBJFILES) \
	      '${LIBM}' '${LIBC}' '${LIBGCC}'

${OBJDIR}/%.bin: ${OBJDIR}/%.axf
	@echo "  Generating binary   ${@} ";
	${OBJCOPY} -O binary ${^} ${@}


###############################################################################
#   Targets                                                                   #
###############################################################################

#
# The default rule, which causes the $(PROGNAME) example to be built.
#
all: ${OBJDIR} ${OBJDIR}/$(PROGNAME).bin
	@echo "Done"

#
# Clean out all the generated files
#
clean:
	rm -rf ${OBJDIR} ${wildcard *~} html latex docs  && echo "Done."

#
# Transfer binary to board
#
flash: deploy
burn: deploy
ifeq ($(FLASHER),lm4flash)
deploy: ${OBJDIR}/$(PROGNAME).bin
	sudo ${FLASHER} $^
else
deploy: ${OBJDIR}/$(PROGNAME).bin $(OPENOCDFLASHSCRIPT)
	sudo ${FLASHER} -f $(OPENOCDBOARD) -f $(OPENOCDFLASHSCRIPT)
endif

#
# Disassembling
#
disassembly:$(OBJDIR)/$(PROGNAME).dump
dump: disassembly
$(OBJDIR)/$(PROGNAME).dump: $(OBJDIR)/$(PROGNAME).axf
	@echo "  Disassembling       ${^} and storing in $(OBJDIR)/$(PROGNAME).dump"
	$(OBJDUMP) $(ODFLAGS) $^ > $(OBJDIR)/$(PROGNAME).dump

#
# List size
#
size: $(OBJDIR)/$(PROGNAME).axf
	$(OBJSIZE) $^

#
# List size
#
nm: $(OBJDIR)/$(PROGNAME).axf
	$(OBJNM) $^

#
# The rule to create the target directory.
#
${OBJDIR}:
	mkdir -p ${OBJDIR}

#
# Rules for building the $(PROGNAME) example.
#
${OBJDIR}/$(PROGNAME).bin: ${OBJDIR}/$(PROGNAME).axf
${OBJDIR}/$(PROGNAME).axf: ${OBJFILES}

#
# Open files in editor windows
#
edit:
	$(GUIEDITOR) Makefile *.c *.h *.ld &

#
# Debug command
#
debug: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	$(GDB) $(GDBFLAGS) $(OBJDIR)/$(PROGNAME).axf

$(GDBINIT):
	echo "# Run this script using gdb source command" > $(GDBINIT)
	echo "target remote localhost:3333" >> $(GDBINIT)
	echo "break main" >> $(GDBINIT)
	echo "continue" >> $(GDBINIT)

$(OPENOCDFLASHSCRIPT):
	echo "reset halt" > $(OPENOCDFLASHSCRIPT)
	echo "flash probe 0" >> $(OPENOCDFLASHSCRIPT)
	echo "flash write_image erase ${OBJDIR}/$(PROGNAME).bin 0" >> $(OPENOCDFLASHSCRIPT)
	echo "reset run" >> $(OPENOCDFLASHSCRIPT)

$(OPENOCDDEBUGSCRIPT):
	echo "monitor reset halt" > $(OPENOCDFLASHSCRIPT)
	echo "monitor flash probe 0" >> $(OPENOCDFLASHSCRIPT)
	echo "monitor flash write_image erase ${OBJDIR}/$(PROGNAME).bin 0" >> $(OPENOCDFLASHSCRIPT)
	echo "monitor reset run" >> $(OPENOCDFLASHSCRIPT)

#
# Open a terminal windows
#
terminal: term
term:
	if [ -c $(TTYTERM) ] ; then echo "Starting terminal"; \
	$(TERMAPP) -x "sh -c \" $(TTYPROG) $(TTYPARMS) \" " ; \
	else echo "Could not open $(TTYTERM)"; fi

#
# Generate documentation using doxygen
#
doxygen:
	$(DOXYGEN) *.c *.h
.PHONY: default all clean flash burn deploy disassembly dump size nm edit debug terminal term doxygen

#
# Dependencies
#
-include $(OBJFILES:%.o=%.d)
