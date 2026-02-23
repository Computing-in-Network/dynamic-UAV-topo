FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    curl \
    iproute2 \
    procps \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY docker/run_fds_demo.sh /usr/local/bin/run_fds_demo.sh
RUN chmod +x /usr/local/bin/run_fds_demo.sh

ENTRYPOINT ["/usr/local/bin/run_fds_demo.sh"]
