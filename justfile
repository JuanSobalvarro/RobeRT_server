

default:
    just --list

[group('building')]
build:
    @echo "Building RobeRT..."
    cmake -G Ninja -S . -B build -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
    cmake --build build

[group('building')]
docker_build:
    @echo "Building Docker image..."
    docker build -t robert-middleware:1.0 .

[group('building')]
clean:
    @echo "Cleaning build dir..."
    rm -rf build

[group('run')]
run:
    @echo "Running RobeRT..."
    @./build/robert_server.exe ./config.yaml
    @echo "RobeRT stopped."

[group('run')]
docker_run:
    @echo "Running Docker container..."
    docker run --network host -v ./config/config.yaml:/app/config/config.yaml robert-middleware:1.0

build_run:
    just build
    just run
