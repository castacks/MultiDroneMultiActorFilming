FROM ubuntu:jammy

# COPY src /airlab/src
# COPY example /airlab/example
COPY LICENSE /airlab/
COPY README.md /airlab/
# COPY SubmodularMaximization.jl /airlab/

RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install tar
RUN apt-get -y install wget

RUN apt-get -y install python3-pip

RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir "numpy<2.0"
RUN pip install --no-cache-dir "matplotlib==3.5.2"
RUN pip install --no-cache-dir tikzplotlib
RUN pip install --no-cache-dir notebook

RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.2-linux-x86_64.tar.gz
RUN tar zxvf julia-1.8.2-linux-x86_64.tar.gz

ENV PATH="$PATH:/julia-1.8.2/bin"
ENV JULIA_LOAD_PATH="/airlab:/airlab/src/mdma_greedy:/env:$JULIA_LOAD_PATH"
ENV JULIA_NUM_THREADS=12
ENV DISPLAY=$DISPLAY


COPY juliapackages /env
WORKDIR /env
RUN julia -e 'using Pkg; Pkg.activate("."); Pkg.instantiate();'

COPY blender /airlab/blender
WORKDIR /airlab/blender
RUN wget https://mirror.clarkson.edu/blender/release/Blender4.2/blender-4.2.1-linux-x64.tar.xz
RUN tar xvf /airlab/blender/blender-4.2.1-linux-x64.tar.xz

RUN apt-get -y install build-essential git subversion cmake libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev libxrender-dev libsm-dev
RUN apt-get -y install libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev

ENV PATH="$PATH:/airlab/blender/blender-4.2.1-linux-x64"


WORKDIR /airlab
