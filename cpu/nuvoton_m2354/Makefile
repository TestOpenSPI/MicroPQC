MODULE = cpu

ifeq ($(TRUSTZONE),nonsecure)
NOCSRC = nsc.c nsc_wrap.c
endif

ifneq (,$(filter shell,$(USEMODULE)))
DIRS += _test
endif

include $(BASE)/Makefile.base

