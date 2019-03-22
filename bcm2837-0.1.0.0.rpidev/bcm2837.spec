# Copyright (c) 2019 Wind River Systems, Inc.
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
# Modification history
# --------------------
# 08mar19,hkl  version 0.1.0.0, created (F11409)
#
# DESCRIPTION
# RPM spec file.
#

Name: bcm2837_platform
Group: not-shipped
Prefix: /helix/guests/vxworks-7/pkgs_v2/os/psl/unsupported/bcm2837
Provides: installonlypkg
Requires: infrastructure_platform
Requires: arm_platform

#VX7Name: bcm2837
#VX7Group: arch/arm
#VX7Prefix: /vxworks-7/pkgs_v2/os/psl/unsupported/bcm2837
#VX7Provides: installonlypkg
#VX7Requires: infrastructure
#VX7Requires: arm

Summary: BCM2837 processor support library
Version: 0.1.0.0

BuildArch: noarch

Vendor: Wind River Systems
URL: http://windriver.com
Packager: Wind River <http://www.windriver.com>
License: WindRiver
Distribution: vxworks-7
Release: vx7

AutoReqProv: no

#do not strip out binaries
%global __os_install_post %{nil}

%define _unpackaged_files_terminate_build 0

%description
BCM2837 processor support library

%files
%defattr(-, root, root, 0755)
%{prefix}/*

%changelog
* Fri Dec 28 2018 Wind River 0.1.0.0
- created initial version (F11409)
