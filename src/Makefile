#
# Makefile for Bosch Symmetrical SPI driver
#

ccflags-y := -std=gnu99 -Wno-declaration-after-statement
ifeq ($(CONFIG_BOSCH_SYMSPI_SPI_MASTER), y)
    ccflags-y += -DSYMSPI_SPI_MASTER=true
else ifeq ($(CONFIG_BOSCH_SYMSPI_SPI_MASTER), n)
    ccflags-y += -DSYMSPI_SPI_MASTER=false
endif
ccflags-y +=\
    -DSYMSPI_OUR_FLAG_INACTIVE_STATE_MINIMAL_TIME_USEC=$(CONFIG_BOSCH_SYMSPI_OUR_FLAG_INACTIVE_STATE_MINIMAL_TIME_USEC)

obj-$(CONFIG_BOSCH_SYMSPI) += symspi.o
ifeq ($(CONFIG_BOSCH_SYMSPI_TEST_MODULE), y)
    obj-$(CONFIG_BOSCH_SYMSPI) += symspi_test.o
endif
