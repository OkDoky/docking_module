FROM ros:noetic-ros-core

RUN apt update && apt dist-upgrade -y

COPY ./install.sh /

RUN /install.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
    CMD [ "bash" ]