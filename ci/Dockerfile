# This is the docker image definition used to build FreeCAD. It's currently accessible on:
# https://hub.docker.com/repository/docker/freecadci/runner
# on under name freecadci/runner when using docker

FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-22.04
LABEL MAINTAINER Przemo Firszt

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends gnupg2 && \
	echo "deb http://ppa.launchpad.net/freecad-maintainers/freecad-daily/ubuntu focal main" >> /etc/apt/sources.list.d/freecad-daily.list && \
	apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 83193AA3B52FF6FCF10A1BBF005EAE8119BB5BCA && \
    rm -rf /var/lib/apt/lists/*

# Main set of FreeCAD dependencies. To be verified.
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends  \
    cmake \
    cmake-gui \
    doxygen \
    graphviz \
    libboost-date-time-dev \
    libboost-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-iostreams-dev \
    libboost-program-options-dev \
    libboost-python-dev \
    libboost-regex-dev \
    libboost-serialization-dev \
    libboost-thread-dev \
    libcoin-dev \
    libeigen3-dev \
    libgts-bin \
    libgts-dev \
    libkdtree++-dev \
    libmedc-dev \
    libocct-data-exchange-dev \
    libocct-ocaf-dev \
    libocct-visualization-dev \
    libopencv-dev \
    libproj-dev \
    libpyside2-dev \
    libqt5opengl5-dev \
    libqt5svg5-dev \
    libqt5x11extras5-dev \
    libqt5xmlpatterns5-dev \
    libshiboken2-dev \
    libspnav-dev \
    libvtk7-dev \
    libx11-dev \
    libxerces-c-dev \
    libzipios++-dev \
    occt-draw \
    pyside2-tools \
    python3-dev \
    python3-git \
    python3-markdown \
    python3-matplotlib \
    python3-packaging \
    python3-pivy \
    python3-ply \
    python3-pyside2.qtcore \
    python3-pyside2.qtgui \
    python3-pyside2.qtnetwork \
    python3-pyside2.qtsvg \
    python3-pyside2.qtwebchannel \
    python3-pyside2.qtwebengine \
    python3-pyside2.qtwebenginecore \
    python3-pyside2.qtwebenginewidgets \
    python3-pyside2.qtwidgets \
    pyqt5-dev-tools \
    qtbase5-dev \
    qtbase5-dev-tools \
    qttools5-dev \
    qtwebengine5-dev \
    swig \
	&& \
    rm -rf /var/lib/apt/lists/*

# these are for debugging purposes only. Not required to build FreeCAD
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
	bash \
	nano \
	vim \
	python3-pip \
	&& \
    rm -rf /var/lib/apt/lists/*

# DRI and GLX drivers to run within VS Code
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
	&& \
    rm -rf /var/lib/apt/lists/*
