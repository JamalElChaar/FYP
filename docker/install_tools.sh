#!/bin/bash
set -e

echo "---- Installing system tools and libraries ----"

apt-get update && apt-get install -y \
  cmake \
  build-essential \
  lsb-release \
  bash-completion \
  wget \
  unzip \
  curl \
  git \
  htop \
  nano \
  vim \
  python3-dev \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  libncurses-dev \
  mesa-utils \
  x11-apps

echo "---- System tools installed successfully ----"
