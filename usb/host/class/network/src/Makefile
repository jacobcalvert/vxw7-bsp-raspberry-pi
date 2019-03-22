# Makefile - makefile for usb
#
# Copyright (c) 2013, 2019 Wind River Systems, Inc.  
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met: 
# 
# 1) Redistributions of source code must retain the above copyright notice, 
# this list of conditions and the following disclaimer. 
# 
# 2) Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation and/or 
# other materials provided with the distribution. 
# 
# 3) Neither the name of Wind River Systems nor the names of its contributors may be 
# used to endorse or promote products derived from this software without specific 
# prior written permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
#
# modification history
# --------------------
# 29jan19,npc  added Lan78xx usb ethernet card support (F11409)
# 27apr13,j_x  create
#
# DESCRIPTION
# This file contains the makefile rules for building the usb library
#
#*/

LIB_BASE_NAME = usbnetwork

C_SOURCE = usb2Pgs.c usb2End.c usb2Asix.c usb2Dm960x.c

ifeq ($(CPU), ARMARCH8A)
C_SOURCE += usb2Lan78xx.c
endif

ifeq ($(findstring $(VX_CPU_FAMILY), simlinux simpc simso),)
OBJS = $(C_SOURCE:.c=.o)
else
OBJS=
endif

include $(WIND_KRNL_MK)/rules.library.mk




