all: installed

TARBALL = build/osm-gps-map-0.7.2.tar.gz
TARBALL_URL = http://www.johnstowers.co.nz/files/osm-gps-map/osm-gps-map-0.7.2.tar.gz
SOURCE_DIR = build/osm-gps-map-0.7.2
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure
	cd $(SOURCE_DIR) && make
	mkdir -p $(CURDIR)/lib
	cp -r $(CURDIR)/$(SOURCE_DIR)/src/.libs/* $(CURDIR)/lib/
	mkdir -p $(CURDIR)/include
	cp -r $(CURDIR)/$(SOURCE_DIR)/src/*.h $(CURDIR)/include/
	touch installed

clean:
		-rm -rf $(SOURCE_DIR) installed
wipe: clean
		-rm -rf build include lib
