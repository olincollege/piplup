# piplup
Repo for code and documentation related to the 2023-2024 Amazon Robotics SCOPE team at Olin College of Engineering.
![poster-1](https://github.com/user-attachments/assets/47f9c288-f277-459c-a70d-d390a290b3d6)


# Setup and Build Instructions
## Drake
- Install bazel by following: https://bazel.build/install/ubuntu  and ensure that the bazel major version is 6 with bazel --version 

## Kortex

- Download the kortex .whl file from https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl  by right-clicking the 2.6.0 .whl file.

- Install the .whl file by navigating to the downloaded location in terminal and running: pip install [name of .whl file]

- Then when running the teleop_demo, go into all of the collections.MutableMap errors and insert an abcto make it collections.abc.MutableMap

## Oculus Reader
- Follow the git lfs config instructions on the README and then clone this repo: https://github.com/rail-berkeley/oculus_reader/tree/main 
navigate into the repo and run pip install -e .

## ROS2 Humble Install
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 

## Serial Library
`pip install pyserial`

https://github.com/tylerjw/serial
```
git clone https://github.com/tylerjw/serial
cd serial
git checkout ros2
source /opt/ros/humble/setup.bash
[change the /tmp/usr/local to /usr/local in 3rd line of the Makefile]
make
sudo make install
```

## FASTSam
`pip install segment-anything-fast`
`pip install PyYAML`


## Troubleshooting

No such package “@@environ//” or related

Check Bazel version and install 6.1.1 if the version is too new.

sudo apt install bazel-6.1.1
sudo rm /usr/bin/bazel
sudo ln -s /usr/bin/bazel-6.1.1 /usr/bin/bazel
bazel --version
