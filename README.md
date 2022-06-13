# path_sampler
## Description
This package will take `N` number of samples from a trajectory provided in a rosbag file.

A docker compose file is provided to run this package on the docker.

## Usage
1. Put the rosbag file to be sampled to `path_sampler/data`
2. Navigate to folder `path_sampler/launch`
```bash
cd /path/to/path_sampler/launch
```
3. Open `path_sampler/launch/path_sampler.launch` with your favorite editor. Adjust these parameters: `data_path`, `num_of_samples`, `sampled_topic`. For `data_path`, to make it compatible with provided docker-compose file, please change only the filename. For `sampled_topic`, please fill with the topic name that will be sampled.
4. Save the `path_sampler.launch`
2. Navigate to folder `path_sampler`
```bash
cd /path/to/path_sampler/
```
3. Run docker-compose
```bash
docker compose up
```
4. You should see some output in the terminal. After `path_sampler` finishes, in some occasion the container is still running because the roslaunch has not exited. In that case, please Ctrl+C to stop the roslaunch and exit the container. Then, remove the docker container by using
```bash
docker compose down
```
