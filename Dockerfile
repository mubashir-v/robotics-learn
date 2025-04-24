FROM debian:bullseye-slim

# Install mosquitto clients and cleanup
RUN apt-get update && apt-get install -y \
    mosquitto-clients \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Default working directory
WORKDIR /app

# Let the CMD be overridden by workflow (you can define it to run a specific command)
CMD ["bash"]
