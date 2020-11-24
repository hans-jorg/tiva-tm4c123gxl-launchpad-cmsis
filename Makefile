
#
# Makefile to manage all project at once
#

#MAKEFLAGS=--no-print-directory

.SILENT:

default: build

help:
	echo "Choose clean, zip or build"
	exit 0

all: build

clean:
	echo "Cleaning..."
	for i in *; do if [ -d "$$i" ]; then echo "Cleaning $$i ..." ; ( cd $$i;  make clean ); fi; done
	rm -rf *.zip

zip:
	echo "Zipping..."
	for i in *; do if [ -d "$$i" ]; then echo "Zipping $$i ..." ; zip -r $$i.zip $$i; fi; done

build:
	echo "Making all"
	for i in *; do if [ -d "$$i" ]; then echo "Making $$i ..." ; cd $$i; make build ; cd .. ;  fi; done
