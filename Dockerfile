FROM ubuntu:latest

COPY src /airlab/src
COPY example /airlab/example
COPY LICENSE /airlab/
COPY README.md /airlab/
COPY SubmodularMaximization.jl /airlab/

RUN apt-get update
RUN apt-get -y install tar
RUN apt-get -y install wget

RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.2-linux-x86_64.tar.gz
RUN tar zxvf julia-1.8.2-linux-x86_64.tar.gz

ENV PATH="$PATH:/julia-1.8.2/bin"

COPY juliapackages /env
WORKDIR /env
RUN julia -e 'using Pkg; Pkg.activate("."); Pkg.instantiate();'
