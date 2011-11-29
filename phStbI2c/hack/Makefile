KERNEL_DIR := `pwd`/..

obj-m := i2c-algo-IP0105.o

driver:     i2c-algo-IP0105.c
    make -C $(KERNEL_DIR) SUBDIRS=`pwd` modules
