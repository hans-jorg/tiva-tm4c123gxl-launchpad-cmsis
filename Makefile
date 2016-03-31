


default:
	@echo "Use one of the options: all clean zip"
	@exit 0

clean:
	@echo "Cleaning ..."
	@for i in *; do if [ -d "$$i" ]; then echo "Cleaning $$i ..." ; rm -f $$i.zip  ; ( cd $$i;  make clean ); fi; done

zip: clean
	@echo "Zipping ..."
	@for i in *; do if [ -d "$$i" ]; then echo "Zipping $$i ..." ; zip -r $$i.zip $$i ; fi; done

all:
	@echo "Building ..."
	@for i in *; do if [ -d "$$i" ]; then echo "Building $$i ..." ; ( cd $$i;  make ); fi; done
