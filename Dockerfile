FROM ubuntu:20.04

WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
        wget \
        build-essential \
        python3-dev \
        python3-pip \
        python3-setuptools \
        zip \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# GCC ARM
RUN cd .. && \
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    tar xjf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    echo "PATH=$PATH:/gcc-arm-none-eabi-9-2019-q4-major/bin" >> ~/.bashrc && \
    cd /app

ENV PATH="/gcc-arm-none-eabi-9-2019-q4-major/bin:${PATH}"

RUN pip3 install pyserial
RUN pip3 install inquirer
