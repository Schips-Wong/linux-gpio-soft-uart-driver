##############################################################################
#		copyleft @ 2010 2011 Late Lee
# file name:Makefile
# A simple Makefile for device driver by Late Lee from www.latelee.org
# based on LDD3 and other guys works.
#
#
# note:
#      You need to change your module name & obj file(s),and you may
#      also need to change 'KERNELDIR'.
#
# Ref : https://blog.csdn.net/subfate/article/details/6375696
#
##############################################################################

# debug or not
#DEBUG = y
ifeq ($(DEBUG), y)
	DEBFLAGS = -O -g
else
	DEBFLAGS =
endif

EXTRA_CFLAGS += $(DEBFLAGS)
#EXTRA_CFLAGS += -I$(INCDIR)

########## change your module name here
MODULE   = my_soft_uart

########## change your obj file(s) here
$(MODULE)-objs:= softuart_module.o softuart.o softuart_queue.o
#CROSS_COMPILE ?= arm-linux-gnueabihf-
#ARCH 		  ?= arm

ifneq ($(KERNELRELEASE), )
	obj-m := $(MODULE).o

else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

all:
	$(MAKE_BEGIN)
	@echo 
	@if \
	$(MAKE) INCDIR=$(PWD)/configs -C $(KERNELDIR) M=$(PWD) modules; \
	then $(MAKE_DONE);\
	else \
	$(MAKE_ERR);\
	exit 1; \
	fi

#all:
#	$(MAKE_BEGIN)
#	@echo
#	@if \
#	$(MAKE) INCDIR=$(PWD)/configs -C $(KERNELDIR) M=$(PWD) modules; \
#	then $(MAKE_DONE);\
#	cp -v $(MODULE).ko /tmp/tftpboot;\
#	else \
#	$(MAKE_ERR);\
#	exit 1; \
#	fi

endif

show:
	@echo "ARCH     :    ${ARCH}"
	@echo "CC       :    ${CROSS_COMPILE}gcc"
	@echo "KDIR     :    ${KERNELDIR}"
	@echo "$(MODULE):    $(ALLOBJS)"
clean:
	$(CLEAN_BEGIN)
	rm -rf *.cmd *.o *.ko *.mod.c *.symvers *.order *.markers .tmp_versions .*.cmd *~ .*.d
	$(CLEAN_END)

.PHONY:all clean show
#xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
### nothing
#OFFSET=\e[21G    # 21 col
COLOR1=\e[32m  # all --> bule
COLOR2=\e[33m  # clean --> brown
COLOR3=\e[31m  # error --> red
RESET=\e[0m

CLEAN_BEGIN=@echo -e "$(OFFSET)$(COLOR2)Cleaning up...$(RESET)"
CLEAN_END=@echo -e "$(OFFSET)$(COLOR2)Cleaned.$(RESET)"

MAKE_BEGIN=@echo -ne "$(OFFSET)$(COLOR1)Compiling...$(RESET)"
### I do not forget "@", but it DOES NOT need "@"
MAKE_DONE=echo -e "$(OFFSET)$(COLOR1)Compilied.$(RESET)"
MAKE_ERR=echo -e "$(OFFSET)$(COLOR3)[Oops! Error occurred]$(RESET)"
### nothing end here
	#$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} INCDIR=$(PWD)/configs -C $(KERNELDIR) M=$(PWD) modules; \

############# Makefile end here
