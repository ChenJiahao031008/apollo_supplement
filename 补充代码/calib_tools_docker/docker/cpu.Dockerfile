FROM ubuntu:18.04

LABEL maintainer="2942826077@qq.com"

# ------ USER ROOT HAS BEEN ACTIVATED ------

# use root for dependency installation:
USER root

# ------ PART 0: set environment variables ------

# set up environment:
ENV DEBIAN_FRONTEND noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

# ------ PART 1: set up CN sources ------

# Ubuntu:
COPY ${PWD}/docker/image/etc/apt/sources.list /etc/apt/sources.list
RUN rm -f /etc/apt/sources.list.d/*

# Python:
COPY ${PWD}/docker/image/etc/pip.conf /root/.pip/pip.conf

# ------ PART 2: set up apt-fast -- NEED PROXY DUE TO UNSTABLE CN CONNECTION ------

# install:
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
    # PPA utilities:
    software-properties-common \
    # certificates management:
    dirmngr gnupg2 \
    # download utilities:
    axel aria2 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 1EE2FF37CA8DA16B && \
    add-apt-repository ppa:apt-fast/stable && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated apt-fast && \
    rm -rf /var/lib/apt/lists/*

# CN config:
COPY ${PWD}/docker/image/etc/apt-fast.conf /etc/apt-fast.conf

# ------ PART 3: add external repositories ------
# libsparse:
RUN add-apt-repository -r ppa:bzindovic/suitesparse-bugfix-1319687

# ------ PART 4: install packages ------

RUN apt-fast update --fix-missing && \
    apt-fast install -y --no-install-recommends --allow-unauthenticated \
        # package utils:
        sudo dpkg pkg-config apt-utils \
        # security:
        openssh-server pwgen ca-certificates \
        # network utils:
        curl wget iputils-ping net-tools \
        # command line:
        vim grep sed patch \
        # io:
        pv zip unzip bzip2 \
        # version control:
        git mercurial subversion \
        # daemon & services:
        supervisor nginx \
        # dev. tools:
        terminator \
        # dev. tools:
        terminator \
        # potential image & rich text IO:
        libsdl1.2-dev libsdl-net1.2-dev libsdl-image1.2-dev \
        lxde \
        gnome-themes-standard \
        xvfb dbus-x11 x11-utils libxext6 libsm6 x11vnc \
        gtk2-engines-pixbuf gtk2-engines-murrine pinta ttf-ubuntu-font-family \
        mesa-utils libgl1-mesa-dri libxrender1 \
        gnuplot \
        texlive-latex-extra \
        #
        # general development:
        #
        # a. c++:
        gcc-9 g++-9 \
        make cmake build-essential autoconf automake libtool \
        libglib2.0-dev libboost-dev libboost-all-dev \
        libomp-dev libtbb-dev \
        libgoogle-glog-dev \
        # c. lua:
        lua5.3 liblua5.3-dev libluabind-dev \
        #
        # numerical optimization:
        #
        libeigen3-dev \
        #
        # 3D graphics:
        #
        freeglut3-dev \
        libqt4-dev libqt4-opengl-dev \
        qt5-default qt5-qmake \
        qtdeclarative5-dev libqglviewer-dev-qt5 \
        #
        # base lib
        #
        libopencv-dev libpcl-dev libceres-dev libyaml-cpp-dev proj-bin && \
    apt-fast autoclean && \
    apt-fast autoremove && \
    rm -rf /var/lib/apt/lists/*

# enable dependency lib linking:
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100

# ------ PART 5: offline installers ------

# load installers:
COPY ${PWD}/docker/installers /tmp/installers
WORKDIR /tmp/installers

# install ceres:
RUN git clone https://github.com/ceres-solver/ceres-solver.git -o ceres-solver && \
    cd ceres-solver && git checkout 1.14.x && cd .. &&\
    mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver && \
    make -j8 && make install

RUN wget https://cmake.org/files/v3.15/cmake-3.15.0-Linux-x86_64.tar.gz && \
    tar -zxvf cmake-3.15.0-Linux-x86_64.tar.gz && \
    mv cmake-3.15.0-Linux-x86_64/ /opt/cmake-3.15.0 && \
    ln -sf /opt/cmake-3.15.0/bin/*  /usr/bin/ && \
    rm -rf cmake-3.15.0-Linux-x86_64.tar.gz

# install tini:
RUN chmod u+x ./download-tini.sh && ./download-tini.sh && dpkg -i tini.deb && \
    apt-get clean

RUN rm -rf /tmp/installers

COPY docker/image /

WORKDIR /

# ------------------ DONE -----------------------

ENV LD_LIBRARY_PATH=/usr/local/lib

ENTRYPOINT ["/startup.sh"]
