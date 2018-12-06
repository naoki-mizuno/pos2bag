# pos2bag

Convert RTKLIB post-processing solution files (`.pos` files) to ROS bag files.
`.pos` files contain information regarding the latitude, longitude, height at
a certain timestamp, along with the fix quality, number of satellites, etc.

## Example usage

```console
$ rosrun pos2bag convert.py input.pos output.bag
$ rosbag info output.bag
path:        output.bag
version:     2.0
duration:    53:41s (3221s)
start:       Nov 15 2018 12:32:27.50 (1542252747.50)
end:         Nov 15 2018 13:26:09.00 (1542255969.00)
size:        13.8 MB
messages:    96279
compression: none [17/17 chunks]
types:       geometry_msgs/PointStamped [c63aecb41bfdfd6b7e1fac37c7cbe7bf]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
             tf2_msgs/TFMessage         [94810edda583a504dfda3829e70d7eec]
topics:      /fix          32093 msgs    : sensor_msgs/NavSatFix
             /gnss_point   32093 msgs    : geometry_msgs/PointStamped
             /tf           32093 msgs    : tf2_msgs/TFMessage
```

For more information and options, please check the output of `rosrun pos2bag
convert.py --help`.


## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
