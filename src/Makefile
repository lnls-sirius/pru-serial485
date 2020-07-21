install-library: 
	@echo "......................................................."; \
	echo ".......... Building and installing C library .........."; \
	echo "......................................................."; \
	cd library; \
	gcc -mfloat-abi=hard -Wall -fPIC -O2 -mtune=cortex-a8 -march=armv7-a -I/usr/include -c -o PRUserial485.o PRUserial485.c; \
	ar -rv libPRUserial485.a PRUserial485.o; \
	gcc -shared -Wl,-soname, -o libPRUserial485.so PRUserial485.o; \
	install -m0755 libPRUserial485.a libPRUserial485.so /usr/lib; \
	ldconfig -n /usr/lib/libPRUserial485.*; \
	install -m0755 PRUserial485.h /usr/include; \
	rm PRUserial485.o libPRUserial485.so libPRUserial485.a; \
	echo ".........................................................."; \
	echo ".......... Building and installing PRU firmware .........."; \
	echo ".........................................................."; \
	pasm -V3 -b PRUserial485.p; \
	mv PRUserial485.bin /usr/bin; \
	echo "..........................................................."; \
	echo ".......... Building and installing Python module .........."; \
	echo "..........................................................."; \
	python-sirius setup.py install; \
	rm -rf build/ dist/ PRUserial485.egg-info/

verify-install:
	@$(eval TESTE=$(shell python-sirius -c 'import importlib;exit() if importlib.find_loader("PRUserial485") is None else None; import PRUserial485;print(PRUserial485.__version__())')) \
	echo ""; echo "....................................................................................."; \
	echo ".......... PRUserial485 v$(TESTE) successfully built and installed ;) .........."; \
	echo "....................................................................................."; echo ""

verify-uninstall:
	@$(eval TESTE=$(shell python-sirius -c 'import importlib;exit() if importlib.find_loader("PRUserial485") is None else None; import PRUserial485;print(PRUserial485.__version__())')) \
	echo "---------------------------------------------"; echo "---- Removing PRUserial485 v$(TESTE)"; \
	echo "---------------------------------------------"; echo ""

uninstall-library:
	@echo "-------------------------------"; \
	echo "---- Uninstalling C library"; \
	rm -rf /usr/lib/libPRUserial485.*; \
	rm -rf /usr/include/PRUserial485.h; \
	echo "Ok.";\
	echo "--------------------------------"; \
	echo "---- Uninstalling PRU firmware"; \
	rm -rf /usr/bin/PRUserial485.bin; \
	echo "Ok.";\
	echo "--------------------------------"; \
	echo "---- Uninstalling Python module"; \
	echo "Ok.";\
	rm -rf /usr/local/lib/python3.*/site-packages/PRUserial485*

reinstall: uninstall install

install: install-library verify-install

uninstall: verify-uninstall uninstall-library
