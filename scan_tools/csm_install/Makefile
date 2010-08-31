all: installed

GIT_DIR = build/csm-git
GIT_URL = http://github.com/AndreaCensi/csm.git
GIT_REVISION = 23703a998fff57250322

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	cd $(GIT_DIR) && mkdir -p deploy
	cd $(GIT_DIR) && cmake -DCMAKE_INSTALL_PREFIX:PATH=$(CURDIR)
	cd $(GIT_DIR) && make
	cd $(GIT_DIR) && make install
	touch installed

clean:
	-cd $(GIT_DIR) && make clean
	rm -rf $(CURDIR)/lib $(CURDIR)/include $(CURDIR)/bin installed 

wipe: clean
	rm -rf $(GIT_DIR)

