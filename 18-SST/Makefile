##
#  @file     Makefile
#  @brief    General makefile for TM4C123
#  @version  V1.0
#  @date     31/10/2016
#
#  @note     CMSIS library used
#
#  @note  make options
#
#  @param build        generate image file in $(OBJDIR)
#  @param flash        write to flash using $(FLASHER)
#  @param clean        delete all generated files
#  @param disassembly  disassemble with output file into $(OBJDIR)/$(PROGNAME).S
#  @param dump         generate a hexadecimal dump file into $(OBJDIR)/$(PROGNAME).dump
#  @param nm           list symbol table in standard output
#  @param size         list size of output file
#  @param term         open a terminal for serial communication to board
#  @param openocd      start an openocd proxy
#  @param gdbserver         start debug daemon (Start before debug session)
#  @param debug             enter a debug session (one of below)
#  @param  gdb              enter a debug session using gdb
#  @param  ddd              enter a debug session using ddd (GUI)
#  @param  nemiver          enter a debug session using nemiver (GUI)
#  @param  tui              enter a debug session using gdb in text UI
#  @param doxygen       generate doc files (alias=docs)
#  @param help          print options (default)

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
# Main target
#
#default: usage
default: build

#
# Verbose output for Make (Coment to have verbose output)
#
#MAKEFLAGS+= --silent --no-print-directory
.SILENT:

#
# Folder for object files
#
OBJDIR=gcc

#
# For gcc-arm-embedded
#

#--------------------------------------------------------------------------
#|   ARM core | Command Line Options                       | multilib     |
#| / ARM arch |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M0+ | -mthumb -mcpu=cortex-m0plus                | armv6-m      |
#| Cortex-M0  | -mthumb -mcpu=cortex-m0                    |              |
#| Cortex-M1  | -mthumb -mcpu=cortex-m1                    |              |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv6-m                     |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M3  | -mthumb -mcpu=cortex-m3                    | armv7-m      |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv7-m                     |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M4  | -mthumb -mcpu=cortex-m4                    | armv7e-m     |
#| (No FP)    |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m                    |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M4  | -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp | armv7e-m     |
#| (Soft FP)  | -mfpu=fpv4-sp-d16                          | /softfp      |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m -mfloat-abi=softfp |              |
#|            | -mfpu=fpv4-sp-d16                          |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M4  | -mthumb -mcpu=cortex-m4 -mfloat-abi=hard   | armv7e-m     |
#| (Hard FP)  | -mfpu=fpv4-sp-d16                          | /fpu         |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m -mfloat-abi=hard   |              |
#|            | -mfpu=fpv4-sp-d16                          |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M7  | -mthumb -mcpu=cortex-m7                    | armv7e-m     |
#| (No FP)    |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m                    |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M7  | -mthumb -mcpu=cortex-m7 -mfloat-abi=softfp | armv7e-m     |
#| (Soft FP)  | -mfpu=fpv5-sp-d16                          | /softfp      |
#|            |--------------------------------------------| /fpv5-sp-d16 |
#|            | -mthumb -march=armv7e-m -mfloat-abi=softfp |              |
#|            | -mfpu=fpv5-sp-d16                          |              |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -mcpu=cortex-m7 -mfloat-abi=softfp | armv7e-m     |
#|            | -mfpu=fpv5-d16                             | /softfp      |
#|            |--------------------------------------------| /fpv5-d16    |
#|            | -mthumb -march=armv7e-m -mfloat-abi=softfp |              |
#|            | -mfpu=fpv5-d16                             |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M7  | -mthumb -mcpu=cortex-m7 -mfloat-abi=hard   | armv7e-m     |
#| (Hard FP)  | -mfpu=fpv5-sp-d16                          | /fpu         |
#|            |--------------------------------------------| /fpv5-sp-d16 |
#|            | -mthumb -march=armv7e-m -mfloat-abi=hard   |              |
#|            | -mfpu=fpv5-sp-d16                          |              |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -mcpu=cortex-m7 -mfloat-abi=hard   | armv7e-m     |
#|            | -mfpu=fpv5-d16                             | /fpu         |
#|            |--------------------------------------------| /fpv5-d16    |
#|            | -mthumb -march=armv7e-m -mfloat-abi=hard   |              |
#|            | -mfpu=fpv5-d16                             |              |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.base                | armv8-m.base |
#| Baseline   |                                            |              |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.main                | armv8-m.main |
#| Mainline   |                                            |              |
#| (No FP)    |                                            |              |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.main                | armv8-m.main |
#| Mainline   | -mfloat-abi=softfp -mfpu=fpv5-sp-d16       | /softfp      |
#| (Soft FP)  |                                            | /fpv5-sp-d16 |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -march=armv8-m.main                | armv8-m.main |
#|            | -mfloat-abi=softfp -mfpu=fpv5-d16          | /softfp      |
#|            |                                            | /fpv5-d16    |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.main                | armv8-m.main |
#| Mainline   | -mfloat-abi=hard -mfpu=fpv5-sp-d16         | /fpu         |
#| (Hard FP)  |                                            | /fpv5-sp-d16 |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -march=armv8-m.main                | armv8-m.main |
#|            | -mfloat-abi=hard -mfpu=fpv5-d16            | /fpu         |
#|            |                                            | /fpv5-d16    |
#|------------|--------------------------------------------|--------------|
#| Cortex-R4  | [-mthumb] -march=armv7-r                   | armv7-ar     |
#| Cortex-R5  |                                            | /thumb       |
#| Cortex-R7  |                                            |              |
#| Cortex-R8  |						  |		 |
#| (No FP)    |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-R4  | [-mthumb] -march=armv7-r -mfloat-abi=softfp| armv7-ar     |
#| Cortex-R5  | -mfpu=vfpv3-d16                            | /thumb       |
#| Cortex-R7  |                                            | /softfp      |
#| Cortex-R8  |                                            |              |
#| (Soft FP)  |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-R4  | [-mthumb] -march=armv7-r -mfloat-abi=hard  | armv7-ar     |
#| Cortex-R5  | -mfpu=vfpv3-d16                            | /thumb       |
#| Cortex-R7  |                                            | /fpu         |
#| Cortex-R8  |                                            |              |
#| (Hard FP)  |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-A*  | [-mthumb] -march=armv7-a                   | armv7-ar     |
#| (No FP)    |                                            | /thumb       |
#|------------|--------------------------------------------|--------------|
#| Cortex-A*  | [-mthumb] -march=armv7-a -mfloat-abi=softfp| armv7-ar     |
#| (Soft FP)  | -mfpu=vfpv3-d16                            | /thumb       |
#|            |                                            | /softfp      |
#|------------|--------------------------------------------|--------------|
#| Cortex-A*  | [-mthumb] -march=armv7-a -mfloat-abi=hard  | armv7-ar     |
#| (Hard FP)  | -mfpu=vfpv3-d16                            | /thumb       |
#|            |                                            | /fpu         |
#--------------------------------------------------------------------------

#
# Set the compiler CPU/FPU options.
#
CPUFLAGS=-mcpu=cortex-m4

#
# FPU Options
# ATTENTION!!! Do no mix them. All object files must be compiled with same FPU flags
#
# float-abi options
#        soft:   Full software floating point using integer registers for function parameters
#        hard:   Full hardware floating point
#        softfp: Use hardware floating point with the same abi of software floating point
#
# fpu options
#        fpv?:   ARM FPU (version 1 and 2 deprecated)
#        neon:   ARM Vector FPU
#        crypto: Crypto Unit
#        d16:    Register bank as 16 double precision registers
#        sp:     Single precision
#
#
# ENABLE_FPU is used in start code to initialize FPU
#
#
# To use no FPU, do not define FPUFLAGS
# Use hardware FPU
     #FPUFLAGS=-mfpu=fpv4-sp-d16 -mfloat-abi=hard -DENABLE_FPU
# Use software for Floating Point
     #FPUFLAGS=-mfpu=fpv4-sp-d16 -mfloat-abi=soft
# Use hardware (when available?) but with an interface similar to the used by software
     #FPUFLAGS=-mfpu=fpv4-sp-d16 -mfloat-abi=softfp  -DENABLE_FPU


#
# Specs script (modification of compiler and linker flags)
#
# This parameter is only recognized by gcc.
# Linking must be done by a gcc call instead of a ld call
#
SPECFLAGS= --specs=nano.specs

#
# Serial terminal communication
#
TTYTERM=/dev/ttyACM0
TTYBAUD=9600

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
# The command to flash the device
#
FLASHER=lm4flash
#FLASHER=openocd

#
# gdbserver command
#
GDBSERVER=openocd
#GDBSERVER=lmicdi
OPENOCDDIR=/opt/openocd

#
# Terminal application (used to open new windows in debug)
#
#TERMAPP=xterm
TERMAPP=gnome-terminal

#
# GUI Editor
#
GUIEDITOR=gedit
#GUIEDITOR=geany

#
# Location of additional libraries
#



###############################################################################
# Generally it is not needed to modify the lines bellow                       #
###############################################################################

###############################################################################
#   Commands                                                                  #
###############################################################################


#
# The command for calling the compiler.
#
CC=${PREFIX}-gcc

#
# The command for calling the library archiver.
#
AR=${PREFIX}-ar

#
# The command for extracting images from the linked executables.
#
OBJCOPY=${PREFIX}-objcopy

#
# The command for disassembly
#
OBJDUMP=${PREFIX}-objdump

#
# The command for listing size of code
#
OBJSIZE=${PREFIX}-size

#
# The command for listing symbol table
#
OBJNM=${PREFIX}-nm

#
# Debugger
#
GDB=$(PREFIX)-gdb

#
# GDB servers
#
OPENOCD=openocd
LMICDI=lmicdi



###############################################################################
#   Command parameters                                                        #
###############################################################################

#
# The flags passed to the debugger
#
GDBFLAGS=-x $(GDBINIT) -n

#
# GDB initialization file
#
GDBINIT=$(OBJDIR)/gdbinit

#
# Flags for disassembler
#
ODDISFLAGS= -x -S -D
ODDUMPFLAGS= -x -s

#
# Parameters for OpenOCD
#
OPENOCDBOARD=$(OPENOCDDIR)/share/openocd/scripts/board/ek-tm4c123gxl.cfg
OPENOCDFLASHSCRIPT=$(OBJDIR)/flash.ocd

###############################################################################
# Compilation parameters                                                      #
###############################################################################


#
# Object files
#
OBJFILES=$(addprefix ${OBJDIR}/,$(SRCFILES:.c=.o))

#
# Include path
#
CMSISDEVINCDIR=$(CMSISDIR)/Device/TI/TM4C/Include
CMSISINCDIR=${CMSISDIR}/Include
INCLUDEPATH=${CMSISDEVINCDIR} ${CMSISINCDIR}

#
# The command for calling the linker
# Using gcc instead of ld to use --specs parameters
# To try:
#     -ffunction-sections
#     -fdata-sections
#     -fno-builtin
#     -nostdlib
#     -nodefaultlibs
#  ???
#
LD=${PREFIX}-gcc $(CFLAGS) -nostartfiles -Wl,-Map,$(OBJDIR)/$(PROGNAME).map -Wl,$(LDFLAGS)
#LD=${PREFIX}-ld -Map $(OBJDIR)/$(PROGNAME).map $(LDFLAGS)

#
#
# The flags passed to the assembler.
#
AFLAGS=-mthumb \
       ${CPUFLAGS}  \
       ${FPUFLAGS}  \
       $(admdprefix -I ,${INCLUDEPATH})           \

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
       -DPART_${PART} \
       $(SPECFLAGS)


#
# The flags passed to the linker.
#
# For now only one parameter when using gcc to link
#
LDFLAGS=--gc-sections

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
	${LD}   -T '${LINKERSCRIPT}' \
	        --entry '${ENTRY}' \
	        -o ${@} \
	         $(OBJFILES) \
	         -lm -lc -lgcc
	         # \
	         # ${LIBM} ${LIBC} ${LIBGCC}

${OBJDIR}/%.bin: ${OBJDIR}/%.axf
	@echo "  Generating binary   ${@} ";
	${OBJCOPY} -O binary ${^} ${@}


###############################################################################
#   Targets                                                                   #
###############################################################################

#
# Help menu: Default target
#
help: usage
usage:
	@echo "Usage:"
	@echo "make build        generate image file in $(OBJDIR)"
	@echo "make flash        write to flash using $(FLASHER)"
	@echo "make clean        delete all generated files"
	@echo "make disassembly  disassemble with output file into $(OBJDIR)/$(PROGNAME).S"
	@echo "make dump         generate a hexadecimal dump file into $(OBJDIR)/$(PROGNAME).dump"
	@echo "make nm           list symbol table in standard output"
	@echo "make size         list size of output file"
	@echo "make term         open a terminal for serial communication to board"
	@echo "make gdbserver    start debug daemon (Start before debug session)"
	@echo "make debug        enter a debug session (one of below)"
	@echo "make  gdb         enter a debug session using gdb"
	@echo "make  ddd         enter a debug session using ddd (GUI)"
	@echo "make  nemiver     enter a debug session using nemiver (GUI)"
	@echo "make  tui         enter a debug session using gdb in text UI"
	@echo "doxygen           generate doc files (alias=docs)"
	@echo "help              print options (default)"

#
# The default rule, which causes the $(PROGNAME) example to be built.
#
build: ${OBJDIR} ${OBJDIR}/$(PROGNAME).bin
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
	sudo ${OPENOCD} -f $(OPENOCDBOARD) -f $(OPENOCDFLASHSCRIPT)
endif

#
# Disassembling/Dump
#
dis: disassembly
disassembly: $(OBJDIR)/$(PROGNAME).axf
	@echo "  Disassembling ${^} and storing output in $(OBJDIR)/$(PROGNAME).S"
	$(OBJDUMP) $(ODDISFLAGS) $^ > $(OBJDIR)/$(PROGNAME).S

dump: $(OBJDIR)/$(PROGNAME).axf
	@echo "  Dumping ${^} and storing output in $(OBJDIR)/$(PROGNAME).dump"
	$(OBJDUMP) $(ODDUMPFLAGS) $^ > $(OBJDIR)/$(PROGNAME).dump
imagedump: $(OBJDIR)/$(PROGNAME).bin
	od -x $^ > $(OBJDIR)/$(PROGNAME).hexdump

#
# List size
#
size: $(OBJDIR)/$(PROGNAME).axf
	$(OBJSIZE) -A -x $^

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
debug: gdb
gdb: $(OBJDIR)/$(PROGNAME).axf $(GDBINIT) openocd
	$(GDB) $(GDBFLAGS) $(OBJDIR)/$(PROGNAME).axf

#
# iDebug command with text UI
#
tui: $(OBJDIR)/$(PROGNAME).axf $(GDBINIT)
	$(GDB) -tui $(GDBFLAGS) $(OBJDIR)/$(PROGNAME).axf

#
# iDebug command with text UI
#
cgdb: $(OBJDIR)/$(PROGNAME).axf $(GDBINIT)
	cgdb -d `which $(GDB)`  -x $(OBJDIR)/gdbinit $(OBJDIR)/$(PROGNAME).axf


#
# Debug using GUI
#
ddd: $(OBJDIR)/$(PROGNAME).axf $(GDBINIT)
	ddd --debugger "$(GDB) $(GDBFLAGS)" $(OBJDIR)/$(PROGNAME).axf

#
# Debug using kdbg GUI
#
#kdbg: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
#	kdbg  -r localhost:4242 $(OBJDIR)/$(PROGNAME).axf

#
# Debug using nemiver GUI
#
nemiver: $(OBJDIR)/$(PROGNAME).axf $(GDBINIT)
	nemiver  --remote=localhost:4242  --gdb-binary=`which $(GDB)`     $(OBJDIR)/$(PROGNAME).axf

#
# Start debug daemon (gdb proxy)
#
gdbserver: $(GDBSERVER)
openocd:
	if [ "`pidof openocd`" != "" ]; then kill `pidof openocd`; fi
	if [ -c $(TTYTERM) ] ; \
	then \
	    echo "Starting OpenOCD"; \
	    $(TERMAPP) -e "sh -c \" $(OPENOCD) -f $(OPENOCDBOARD) \" " ; \
	else \
	    echo "Could not open $(OPENOCD)"; fi

lmicdi:
	if [ "`pidof lmicdi`" != "" ]; then kill `pidof lmicdi`; fi
	if [ -c $(TTYTERM) ] ; \
	then \
	    echo "Starting lmicdi"; \
	    $(TERMAPP) -e "sh -c \" $(LMICDI)  \" " ; \
	else \
	    echo "Could not open $(OPENOCD)"; fi

#
# Create gdbinit (debugger initialization scripts)
#
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
# OBS: Remove or disable modemmanager in case of busy device
#
terminal: term
term:
	if [ -c $(TTYTERM) ] ; then echo "Starting terminal"; \
	$(TERMAPP) -e "sh -c \" $(TTYPROG) $(TTYPARMS) \" " ; \
	else echo "Could not open $(TTYTERM)"; fi

#
# Generate documentation using doxygen
#
docs: doxygen
doxygen:
	$(DOXYGEN) *.c *.h

#
# These labels are not files !!!
#

.PHONY: default all clean flash burn deploy disassembly dump size nm edit debug terminal
.PHONY: term doxygen docs

#
# Dependencies
#
-include $(OBJFILES:%.o=%.d)
