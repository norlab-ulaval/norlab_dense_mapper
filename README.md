# norlab_dense_mapper
A 2-D/3-D dense mapping library based on [norlab_icp_mapper](https://github.com/norlab-ulaval/norlab_icp_mapper) intended to be used with the ROS package [norlab_dense_mapper_ros](https://github.com/norlab-ulaval/norlab_dense_mapper_ros).

## How to install

1. Follow the instructions to install the [libpointmatcher](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/index.md#compilation-) library and its dependencies

1. Clone this repository `git clone https://github.com/norlab-ulaval/norlab_dense_mapper.git`

1. `cd norlab_dense_mapper`

1. `mkdir build && cd build`

1. `cmake -DCMAKE_BUILD_TYPE=Release ..`

1. `sudo make install`


## How to use

Next, follow the instruction of the [norlab_dense_mapper_ros](https://github.com/norlab-ulaval/norlab_dense_mapper_ros) package in order to use this library.
