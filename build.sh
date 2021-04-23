#!/bin/bash
# Copyright (c) 2020 TOYOTA MOTOR CORPORATION
# Copyright (c) 2021 MID Academic Promotions, Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set -e

BASE_IMAGE="ghcr.io/hsr-project/hsrb_base_binary"
IMAGE_NAME="ghcr.io/hsr-project/mbot_robocup_dspl_binary"

BUILD_DATE=`date +%Y%m%d`
BUILD_ARGS=""

if [[ -n "${http_proxy}" ]]; then
    BUILD_ARGS="${BUILD_ARGS} --build-arg http_proxy=${http_proxy}"
fi
if [[ -n "${https_proxy}" ]]; then
    BUILD_ARGS="${BUILD_ARGS} --build-arg https_proxy=${https_proxy}"
fi
if [[ -n "${no_proxy}" ]]; then
    BUILD_ARGS="${BUILD_ARGS} --build-arg no_proxy=${no_proxy}"
fi

docker build ${BUILD_ARGS} --build-arg BASE_IMAGE=${BASE_IMAGE}:latest -t ${IMAGE_NAME}:latest . # -t ${IMAGE_NAME}:${BUILD_DATE}
#docker build ${BUILD_ARGS} --build-arg BASE_IMAGE=${BASE_IMAGE}:forclass -t ${IMAGE_NAME}:forclass . # -t ${IMAGE_NAME}:${BUILD_DATE}
#docker build ${BUILD_ARGS} --build-arg BASE_IMAGE=${IMAGE_NAME}:latest -f Dockerfile.nvidia -t ${IMAGE_NAME}:nvidia . # -t ${IMAGE_NAME}:nvidia-${BUILD_DATE}
