APP              = iaa_lab3
APP_SRCS        += iaa_lab3.c
APP_INC 		+= lib/cpx/inc
APP_SRCS        += lib/cpx/src/cpx.c lib/cpx/src/com.c
APP_CFLAGS 		+= -g -O0 -mno-memcpy -fno-tree-loop-distribute-patterns
APP_CFLAGS 		+= -I. -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

io=uart
PMSIS_OS = freertos
pulpChip = GAP
PULP_APP = iaa_lab3
USE_PMSIS_BSP=1

RUNNER_CONFIG = $(CURDIR)/config.ini

include $(RULES_DIR)/pmsis_rules.mk
