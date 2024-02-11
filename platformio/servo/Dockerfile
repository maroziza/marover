FROM osrf/ros:iron-desktop
ARG USERNAME=vscode

RUN sudo apt update -y && sudo apt upgrade -y
RUN useradd -s /bin/bash -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

RUN sudo apt update -y && sudo apt install -y ros-iron-plotjuggler-ros libgpiod-dev

WORKDIR /home/$USERNAME

# Install Blaze
RUN git clone https://bitbucket.org/blaze-lib/blaze.git
WORKDIR /home/$USERNAME/blaze
RUN cmake . -B build
RUN sudo cmake --build build --target install

# Setup bash
COPY shell/.bashrc /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME
