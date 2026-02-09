# see https://www.freertos.org/

ifeq ($(CPU), x86_64)
export FREERTOSDIR = $(EXTMODIR)/FreeRTOS-Kernel/
FREERTOSREPO=https://github.com/FreeRTOS/FreeRTOS-Kernel/
export FREERTOSREV = V10.4.5
else
export FREERTOSDIR = $(EXTMODIR)/FreeRTOS-Kernel/
FREERTOSREPO=https://bitbucket.org/securityplatforminc/freertos/
endif


all: $(FREERTOSDIR)

$(FREERTOSDIR):
	git clone $(FREERTOSREPO) $(FREERTOSDIR)
ifneq ($(FREERTOSREV), )
	git  -C $(FREERTOSDIR) checkout $(FREERTOSREV)
endif
