FROM ros:humble

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

RUN if id -u $UID ; then userdel `id -un $UID` ; fi

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install some dependencies packages
RUN apt-get update -q && \
    apt-get upgrade -q -y && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    git zsh \
    curl wget \
    libboost-all-dev && \
    apt-get clean

# Install neovim
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.tar.gz && \
    rm -rf /opt/nvim && \
    tar -C /opt -xzf nvim-linux-x86_64.tar.gz && \
    echo 'export PATH=$PATH:/opt/nvim-linux-x86_64/bin' >> /home/$USERNAME/.bashrc && \
    echo 'export PATH=$PATH:/opt/nvim-linux-x86_64/bin' >> /home/$USERNAME/.zshrc

# Setup zsh
RUN touch /home/$USERNAME/.zshrc
RUN chsh -s /bin/zsh $USERNAME
ENV SHELL=/bin/zsh

USER $USERNAME

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.zsh' >> /home/$USERNAME/.zshrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.zsh' >> /home/$USERNAME/.zshrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
