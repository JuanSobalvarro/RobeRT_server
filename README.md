# TODO moves
- MoveJ
- MoveAbsJ
- MoveC
- MoveL

# Windows firewall
```bash
New-NetFirewallRule -DisplayName "RobeRT-RobotStudio-In" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 5000
```

# Take in count
- EtherCat
- ROS2 DDS
- shift from std::stringstream to std::memcpy

# keywords
- Hardware Abstraction Layer (HAL)

# About architectural decisions
- Why separate rapid data types from proto types?
Simply because we need separation of concerns even though we may have the same data duplicated in both. The rapid structures are for binary data transmission, while the proto structures are for api serialization of messages between a user client and the middleware server. This separation allows us to optimize each for its specific use case without one affecting the other. For example, we can change the proto structures for better API design without worrying about breaking the binary communication, and vice versa.

# docker build
```bash
docker build -t robert-middleware:x.x .
```

# docker run
```bash
docker run -d --name robert-middleware -p 42069:42069 robert-middleware:1.0
```

# Docker publish
```bash
docker tag robert-middleware:x.x username/robert-middleware:x.x
docker push username/robert-middleware:x.x
```

# Safety and robot reach
It is important to know that the robot reach is limited by two main factors:
- Physical workspace limits
- Kinematic limits
These means that the robot cannot reach any point outside of its sphere/torus reach, and also a point that essentially is inside the workspace but kinematically unreachable.

# Important about networking
The `--network host` flag when running the docker container works differently on linux and windows.
When using windows, you are not really using windows, you are using WSL2. So the host is WSL not windows.
But when using linux, since it runs natively it "attaches" to the network card directly so it is the host.

Thats why when using windows we uise -p 42069:42069 to map the container port to the host port. but if you are using
linux you can use `--network host` instead and avoid the port mapping (and also the NAT latency).
