FROM ubuntu:18.04
MAINTAINER PaddlePaddle Authors <paddle-dev@baidu.com>

WORKDIR /root
RUN cp -a /etc/apt/sources.list /etc/apt/sources.list.bak \
    && sed -i "s@http://.*security.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list \
    && sed -i "s@https://.*security.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list \
    && sed -i "s@http://.*archive.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list \
    && sed -i "s@https://.*archive.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list \
    && apt-get -y update \
    && DEBIAN_FRONTEND=noninteractive apt install -y tzdata \
    && ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/timezone && echo 'Asia/Shanghai' >/etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata \
    # apt install required libraries
    && DEBIAN_FRONTEND=noninteractive apt-get install -y sudo wget gcc g++ cmake gdb vim git curl libgdbm-dev \
    build-essential zlib1g zlib1g-dev libbz2-dev libsqlite3-dev openssl libssl-dev \
    libxslt1-dev libffi-dev unzip pciutils net-tools \
    pkg-config libglib2.0-dev libdbus-glib-1-dev openssh-server \
    gfortran libblas3 libblas-dev liblapack3 liblapack-dev libopenblas-dev \
    libncursesw5-dev libncurses5-dev libopencv-dev python-opencv \
    # create HwHiAiUser user 
    && groupadd -g 1000 HwHiAiUser && useradd -d /home/HwHiAiUser -m -u 1000 -g 1000 -s /bin/bash HwHiAiUser \
    && sed -i '/^root/a\HwHiAiUser ALL=(ALL:ALL) NOPASSWD: ALL'  /etc/sudoers

# install Python3.7.5
RUN wget --no-check-certificate https://mirrors.huaweicloud.com/python/3.7.5/Python-3.7.5.tgz \
    && tar -zxvf Python-3.7.5.tgz \
    && cd Python-3.7.5 \
    && echo 'SSL=/usr/local/ssl'>>/root/Python-3.7.5/Modules/Setup.dist \
    && echo '_ssl _ssl.c -DUSE_SSL -I$(SSL)/include -I$(SSL)/include/openssl -L$(SSL)/lib -lssl -lcrypto'>>/root/Python-3.7.5/Modules/Setup.dist \
    && ./configure --prefix=/usr/local/python3.7.5 --enable-shared \
    && make clean \
    && make -j \
    && make install \
    && cp /usr/local/python3.7.5/lib/libpython3.7m.so.1.0 /usr/lib64 \
    && cp /usr/local/python3.7.5/lib/libpython3.7m.so.1.0 /usr/lib \
    && rm -rf  /usr/bin/python3.7.5 \
    && rm -rf  /usr/bin/pip3.7.5 \
    && rm -rf  /usr/bin/python3.7 \
    && rm -rf  /usr/bin/pip3.7 \
    && ln -s /usr/local/python3.7.5/bin/python3 /usr/bin/python3.7 \
    && ln -s /usr/local/python3.7.5/bin/pip3 /usr/bin/pip3.7 \
    && ln -s /usr/local/python3.7.5/bin/python3 /usr/bin/python3.7.5 \
    && ln -s /usr/local/python3.7.5/bin/pip3 /usr/bin/pip3.7.5 \
    && rm -rf /usr/bin/python \
    && rm -rf /usr/bin/pip \
    && ln -s /usr/local/python3.7.5/bin/python3 /usr/bin/python \
    && ln -s /usr/local/python3.7.5/bin/pip3  /usr/bin/pip

ENV LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/python3.7.5/bin:${PATH}
ENV CPLUS_INCLUDE_PATH=/usr/local/python3.7.5/include/python3.7:${CPLUS_INCLUDE_PATH}

RUN mkdir ~/.pip \
    && echo "[global]" >>~/.pip/pip.conf \
    && echo "index-url=http://mirrors.aliyun.com/pypi/simple/" >>~/.pip/pip.conf \
    && echo "trusted-host=mirrors.aliyun.com" >>~/.pip/pip.conf \
    && pip3.7 install --upgrade pip \
    && pip3.7 install attrs \
    && pip3.7 install numpy==1.17.2 \
    && pip3.7 install decorator \
    && pip3.7 install sympy \
    && pip3.7 install cffi==1.12.3 \
    && pip3.7 install pyyaml \
    && pip3.7 install pathlib2 \
    && pip3.7 install psutil \
    && pip3.7 install protobuf \
    && pip3.7 install scipy \
    && pip3.7 install requests

ARG CANN_TOOLKIT_NAME=Ascend-cann-toolkit_5.1.RC1.alpha001_linux-x86_64.run

# install ascend CANN
RUN wget --no-check-certificate https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/CANN/5.1.RC1.alpha001/${CANN_TOOLKIT_NAME} \
    && cd /root && chmod 777 $CANN_TOOLKIT_NAME && ./$CANN_TOOLKIT_NAME --install

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Ascend/driver/lib64:/usr/local/Ascend/driver/lib64/stub:/usr/local/Ascend/ascend-toolkit/latest/acllib/lib64:/usr/local/Ascend/ascend-toolkit/latest/atc/lib64
ENV PYTHONPATH=$PYTHONPATH:/usr/local/Ascend/ascend-toolkit/latest/acllib/python/site-packages:/usr/local/Ascend/ascend-toolkit/latest/toolkit/python/site-packages:/usr/local/Ascend/ascend-toolkit/latest/atc/python/site-packages:/usr/local/Ascend/ascend-toolkit/latest/pyACL/python/site-packages/acl
ENV PATH=$PATH:/usr/local/Ascend/ascend-toolkit/latest/atc/ccec_compiler/bin:/usr/local/Ascend/ascend-toolkit/latest/acllib/bin:/usr/local/Ascend/ascend-toolkit/latest/atc/bin
ENV ASCEND_AICPU_PATH=/usr/local/Ascend/ascend-toolkit/latest
ENV ASCEND_OPP_PATH=/usr/local/Ascend/ascend-toolkit/latest/opp
ENV TOOLCHAIN_HOME=/usr/local/Ascend/ascend-toolkit/latest/toolkit

# clean 
RUN rm -rf /root/var \
    && rm -rf /root/$CANN_TOOLKIT_NAME \
    && rm -rf /root/Python-3.7.5* \
    && rm -rf /var/cache/apt