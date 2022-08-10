# Restrict to the target platforms
PLATFORMS_ONLY += nrf
TARGET ?= nrf
BOARD ?=conexio-stratus

# Other NRF boards
# BOARD ?=nrf9160/dk
# BOARD ?=nrf5340/dk/application

CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include

