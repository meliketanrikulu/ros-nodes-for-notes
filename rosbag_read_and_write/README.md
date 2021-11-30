## rosbag_read_and_write Package
This package reads the rosbag files with C++, changes the messages and saves them again. In this application, only the topic name has been changed. However, in different applications, you can change the message content and rewrite the rosbag files. </br>
In the application, messages of sensor_msgs/PointCloud2 type were read from rosbag file. You can change the message type for different messages. </br>

This package is developed using ```noetic``` distribution.

### How does it work
``` rosrun rosbag_read_and_write rosbag_read_and_write_node ```
