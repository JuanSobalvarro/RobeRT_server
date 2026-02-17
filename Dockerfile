# Compilation stage
FROM debian:bookworm-slim AS builder

# Install compilers and protobuf
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    protobuf-compiler \
    libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

# Build the project
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Runtime stage
FROM debian:bookworm-slim

# Install only the runtime library for protobuf (very small)
RUN apt-get update && apt-get install -y \
    libprotobuf32 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy only the executable from the builder stage
COPY --from=builder /app/build/egm_server .

# Expose the UDP port
EXPOSE 6510/udp

# Run
CMD ["./egm_server"]