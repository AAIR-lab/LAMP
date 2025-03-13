ARG IMAGENAME

FROM $IMAGENAME

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -l --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME