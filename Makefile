
MAKEFLAGS=--no-print-directory
.SILENT:


default:
	echo "Choose clean, zip or all"
	exit 0

clean:
	echo "Cleaning..."
	for i in *; do if [ -d "$$i" ]; then echo "Cleaning $$i ..." ; ( cd $$i;  make clean ); fi; done
	rm -rf *.zip

zip:
	echo "Zipping..."
	for i in *; do if [ -d "$$i" ]; then echo "Zipping $$i ..." ; zip -r $$i.zip $$i; fi; done

all:
	echo "Making all"
	for i in *; do if [ -d "$$i" ]; then echo "Making $$i ..." ; cd $$i; make $(MAKEFLAGS); cd .. ;  fi; done
