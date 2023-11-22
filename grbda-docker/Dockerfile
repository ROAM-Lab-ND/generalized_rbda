# Base image: Ubuntu 18.04 LTS for hardware compatibility with robots
FROM ubuntu:18.04

LABEL maintainer="Robotics Optimization and Assitive Mobility Laboratory"

RUN apt-get update && apt-get install -y sudo

# Create new user `docker` and disable 
# password and gecos for later
# --gecos explained well here:
# https://askubuntu.com/a/1195288/635348
RUN adduser --disabled-password \
--gecos '' docker

#  Add new user docker to sudo group
RUN adduser docker sudo

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers

# now we can set USER to the 
# user we just created
USER docker

# we can now run sudo commands 
# as non-root user `docker` without
# password prompt

# Install useful Ubuntu packages
RUN echo "Installing Ubuntu packages: "
RUN sudo apt-get update \
    && sudo apt-get upgrade -y \
    && sudo apt-get install -y lsb-release \
    && sudo apt-get install -y ubuntu-standard \
    && sudo apt-get install -y cmake \
    && sudo apt-get install -y vim \
    && sudo apt install -y zip \
    && sudo apt install -y iproute2 \
    && sudo apt install -y net-tools \
    && sudo apt install -y git \
    && sudo apt-get clean all

# configure git
RUN git config --global --add safe.directory /home/docker/generalized_rbda

# Setup generalized_rbda directories
RUN mkdir /home/docker/generalized_rbda
WORKDIR /home/docker/generalized_rbda/

# Install generalized_rbda dependencies
COPY /scripts ./scripts

# Declare an argument to be used as a cache buster

# ARG CACHEBUST=1
RUN sudo ./scripts/install_dependencies.sh
