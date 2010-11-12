all: installed

GIT_DIR = build/libfreenect-git
GIT_URL = git://git.marcansoft.com/libfreenect.git
#GIT_REVISION = 

include $(shell rospack find mk)/git_checkout.mk

PATCH_0001 = 0001-Fixed-memory-trash.patch

installed: $(GIT_DIR)
	cd $(GIT_DIR) && mkdir -p deploy
	cd $(GIT_DIR) && patch -p0 < ../../patches/$(PATCH_0001) 
	cd $(GIT_DIR) && cmake -DCMAKE_INSTALL_PREFIX:PATH=$(CURDIR)
	cd $(GIT_DIR) && make
	cp -rf $(GIT_DIR)/include $(CURDIR)/include
	cp -rf $(GIT_DIR)/lib $(CURDIR)/lib
	touch installed

clean:
	-cd $(GIT_DIR) && make clean
	rm -rf $(CURDIR)/lib $(CURDIR)/include $(CURDIR)/bin $(CURDIR)/build installed 

wipe: clean
	rm -rf $(GIT_DIR)

