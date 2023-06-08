# Instructions for implementing Publisher/Subscriber

## Goal: get images from duckiebot camera, add filters and publish results to the new topic.

## Initializing the ROS package
* `package.xml`
* `CMakeLists.txt`
* (optional) `launch/<launch_file_name>.launch`
* actual code `src/*.py`

### ROS package
Just add new package inside directory `packages`. Instructions below assume the name `my_package`.

### `package.xml`

Basic template for the `package.xml` can be found in the developer's manual page.
Create `package.xml` file inside `packages/my_package` and copy the template. 
Additionaly, in the `dt-core` repository, all `duckietown` related dependencies are also
mentioned in the file.

```xml
<package>
    <name>your_package_name</name>
    <version>0.1.0</version>
    <description>your description</description>
    <maintainer email="your@email.com">Maintainer's name</maintainer>
    <license>None</license>

    <buildtool_depend>catkin</buildtool_depend>

    <!-- This part is optional -->
    <build_depend>duckietown_msgs</build_depend>
    <build_depend>sensor_msgs</build_depend>
    <build_depend>rospy</build_depend>


    <run_depend>duckietown_msgs</run_depend>
    <run_depend>sensor_msgs</run_depend>
    <run_depend>rospy</run_depend>
</package>
```

### `CMakeLists.txt`
ROS uses catkin to build the packages. Create `CMakeLists.txt` file inside `packages/my_package` The example of this file could also be found in the developer's manual. Additionally, in the `dt-core` repository, all `duckietown` related
dependencies are also mentioned in the file, same as in `package.xml`

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_package) // your package name

find_package(catkin REQUIRED COMPONENTS
    rospy
    <!-- Below part is optional -->
    duckietown_msgs
    sensor_msgs
)

catkin_package()

```

### Launch files
Same, templates are in the developer's manual, not used in this example. Packages will be run
by directly calling the node in the launcher script (shown below)



### Nodes
Add the directory `src` inside `packages/my_package`. Inside `packages/my_package/src`, create
your node file, e.g, `my_node.py`.

We need the following:

* get the compressed images from the camera
* convert the compressed image to more convenient type, e.g, cv2
* apply the desired filters
* compress the filtered image
* publish to the new topic

## Imports
```py
#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
```
Justifying each import:
- `#!/usr..` - shebang to indicate, that the file is a script
- `cv2` - for applying the filters
- `rospy` - to create the publishers/subscribers
- `CvBridge` - class to convert from `CompressedImage` to `cv2`, and vice-versa
- `DTROS` - class each node should extend
- `NodeType` - constructor argument for the node, `NodeType.GENERIC`, generally,
should be fine, in this example `NodeType.PERCEPTION` is used.
- `CompressedImage` - messaging object type for the images.

### Class and constructor
```py

# actual topic name where the camera output is stored.
# Replace 'bot_name' with duckiebots' name
TOPIC_NAME = '/bot_name/camera_node/image/compressed'

class MyNode(DTROS):
    def __init__(self, node_name):
        super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # bridge for converting from/to CompressedImage to/from cv2
        self.bridge = CvBridge()

        # subscriber to the camera output topic. Constructor arguments are following:

        # TOPIC_NAME - topic name to which subscribe, if topic with name TOPIC_NAME
        # doesn't exist, new topic will be created

        # CompressedImage - messaging object type, e.g, in the developer manual example
        # String was used to publish/subscribe for text messages

        # self.callback - callback function which will be invoked on each notify,
        # method self.callback implemented below
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1)

        # publisher to upload filtered images. Constructor arguments are following:

        # 'filtered_images' - name of the new topic
        # CompressedImage - type of the published object
        self.pub = rospy.Publisher('filtered_images', CompressedImage, queue_size=1)
```

### Callback function
add inside class `MyNode` the following:
```py
def callback(self, msg):
    print(f'received message with type ${type(msg)}') # ideally, <class 'CompressedImage'>

    # converting from CompressedImage to cv2. yeah... that's actual name of the method 
    converted_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    # applying the filter, here, canny filter with thresholds 50 and 150
    filtered_img = cv2.Canny(converted_img, 50, 150)

    # converting back to CompressedImage
    compressed_result_img = self.bridge.cv2_to_compressed_imgmsg(img)

    # publishing the result
    self.pub.publish(compressed_result_img)
```

### Main function
No need to call any method of the node, callback will be invoked by the updates from the
camera topic, hence only instantiation and rospy.spin()

```py
if __name__ == '__main__':
    node = MyNode(node_name='my_node') # instantiation
    rospy.spin() # blocks until rospy.is_shutdown()
```

### Making the script executable
After editing the node, run the following command:
```sh
chmod +x my_node.py
``` 

### Updating the launcher script
Now go back to the project root. Inside `launchers/` directory, edit the `default.sh`.
Remove old command under `#launching app` comment, replace with:
```sh
dt-exec rosrun my_package my_node.py
```

### Running on the duckiebot
To build the new image, run: (replace `duckiebot_name` with actual name)
```sh
dts devel build -f -H duckiebot_name.local
```

flag `-f` allows to build with uncommited changes, `-H` builds for the duckiebot.

Running the image:
```sh
dts devel run -H duckiebot_name.local
```

### Viewing the results:
Open a terminal, enter the command:
```sh
dts start_gui_tools
```

Inside the new prompt, enter:
```sh
rqt_image_view
```

GUI window will appear, that shows output of the camera. Click on the dropdown menu,
to choose the source topic. If everything done correctly, newly added `filtered_images`
topic should appear. However, no actual output is shown there.

To ensure, filtered images are being published, quit the `image_view`, and in the same 
`gui_tools` prompt, enter:
```sh
rqt_bag
```

`rqt_bag` is a tool for recording and viewing the contents of the topics. 
One shouldn't complain about terrible ui of the open source application, but it's 
still terrible. To get the content
of `filtered_images`, click on the record button (big red circle on the upper left), 
popup screen will appear, choose the desired topic and recording will start.
To stop the recording, press the red circle again. To view the recording, right click on the
footage, choose `View/Image` and slide the timeline indicator triangle, on the right side
the filtered outputs should appear.