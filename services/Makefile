CFLAGS :=  `pkg-config fuse3 --cflags --libs` -DFUSE_USE_VERSION=30 -D_FILE_OFFSET_BITS=64 -g
LDLIBS := `pkg-config fuse3 --cflags --libs` -lpci

obj := mrvl_main.o mrvl_pcie_vfio.o octboot_net.o

ifdef PHC_SUPPORT
obj += mrvl_cuse_dev.o
CFLAGS +=  $(CFLAGS) -DPHC_SUPPORT=1
endif

daemon_test: $(obj)
	cc $(obj) $(LDLIBS) -o daemon_test

clean:
	rm *.o daemon_test
