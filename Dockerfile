FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    build-essential cmake git libprotobuf-dev protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /app/install/setup.bash ]; then source /app/install/setup.bash; fi" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]