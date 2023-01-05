# The LiDAR Localization Package

This package contains the code that can be used to compute the center of a circle from a set of laser scans that are *presumed to be on that circle*.

Assuming that the scans are noisy, the result *will be off*, it does however compute multiple candidates and averages them to get the best possible result in a deterministic way.


## Example Usage

Assuming that the libraries are included, linked and built correctly:

```c++
#include <iostream>
#include <lidar_loc/include/CircleDetection.hpp>

int main(){
    // load the target radius from your configuration
    const double circle_radius = get_circle_radius();
    // obtain the scan ponits using your code
    std::vector<Eigen::Vector2d> scan_points = get_lidar_scan();
    // initialize the circle detector
    lidar_loc::CircleDetection circle_detection(circle_radius);
    // the circle center function returns an optional result
    const std::optional<Eigen::Vector2d> circle_center = 
                circle_detection.compute_center(scan_points);
    // check if the result has a value
    if(circle_center.has_value()){
        // success, process the result (don't forget to access it as a pointer)
        std::cout << "(" << circle_center->x() << "|" << circle_center->y() << ")\n";
        return 0;
    }
    else{
        // failed to detect a circle
        return 1;
    }
}
```

## Building and Running the Tests

This package uses [GTest](https://google.github.io/googletest/) to testfor correct functionality. The instructions for integrating GTest into your project are listed in the quickstart guides.

Inside this directory, run the following commands to build and run the tests.

```bash
# create a build directory
mkdir build
# generate the build files and run the build
cmake -S . -B build/
cmake --build build/
# change to the build directory and run the tests
cd build
ctest -T test --output-on-failure
```

