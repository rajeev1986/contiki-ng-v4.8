# TARGET ?= simplelink
# BOARD ?= launchpad/cc1312r1

PLATFORMS_ONLY += nrf
TARGET ?= nrf
# BOARDS_ONLY = nrf5340/dk/application
BOARDS_ONLY = nrf9160/dk

CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
