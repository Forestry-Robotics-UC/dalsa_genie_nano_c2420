
# dalsa_genie_nano_c2420

### v0.6 (David Portugal + Maria Eduarda Andrada)
ROS driver for the [Dalsa Genie Nano C2420 Multispectral Camera](https://www.edmundoptics.eu/p/c2420-23-color-dalsa-genie-nano-poe-camera/4059/) using the GigE SDK and OpenCV. The code is based on the C++ SDK example, and it publishes two sensor_msgs/Image topics on ROS (colored and mono) and their compressed counterparts.

The driver also publishes images based on vegetation indexes NDVI, CVI and TVI, and each channel separately. All topics are published raw (sensor_msgs/Image) and compressed (sensor_msgs/CompressedImage).

Tested on Ubuntu 18.04 and ROS Melodic

## Installation

1. Make sure you have the following dependencies: ```sudo apt install ethtool libgtk-3-dev libglade2-0 libglade2-dev```

2. Register and Download the [DALSA GigE-V Framework](https://www.teledynedalsa.com/en/products/imaging/vision-software/linux-gige-v-framework/) for Linux. 
 
    + There's a copy of the v2.10.0.0157 of the framework from May 2019 [here](https://www.dropbox.com/s/617itmz87yzc00b/gige-v-framework_21000157.zip?dl=0), in case you don't want to register.

3. Extract the x86 .tar to your ```HOME``` folder.

4. Run ```sudo ./corinstall``` in the ```~/GigE-V-Framework_2.10.0.0157/DALSA``` directory.

5. Logout and login again into Ubuntu.

6. Connect the camera (in our setup we connect the camera to a switch, and make sure both are in the same ethernet network).

7. Check if you can find the camera on the network:
 
     + ```cd ~/GigE-V-Framework_2.10.0.0157/DALSA/GigeV/tools/GigeDeviceStatus```
 
     + ```./GigeDeviceStatus```
 
     + You should now see the camera on the network:

     ![camera_list](doc/dalsa_genie_cam_list.png)

8. Checking images from the camera using the SDK C++ example:
 
      + ```cd ~/GigE-V-Framework_2.10.0.0157/DALSA/GigeV/examples/genicam_cpp_demo```
 
      + ```make```
 
      + ```./genicam_cpp_demo```
 
      + When the program launches, you'll only see a black window. In the terminal, hit "G" and "Enter" and you'll now see the camera feed.

## Note

You do not need to set a fixed camera IP address in the network. The driver will automatically detect it.

## Compiling

```
cd your_work_space
catkin_make 
```

## Example Usage

### dalsa_genie_nano_c2420



**Parameters**

`dalsa_camera_frame` (`string`, `default: dalsa_link`)

The frame ID entry for the messages.

`dalsa_camera_topic` (`string`, `default: dalsa_camera`)

Topic to publish colored image.

`camera_index` (`int`, `default: 0`)

Only useful for multi-camera setups. You can specify the index of the camera that you want to run.

`publish_mono` (`bool`, `default: false`)

Activate publishing of the monochromatic image topic as well.

`dalsa_camera_mono_topic` (`string`, `default: dalsa_camera_mono`)

Topic to publish mono image (if `publish_mono` is true).

`use_synchronous_buffer_cycling` (`bool`, `default: false`)

Enable/disable buffer full/empty handling (cycling). This allows Asynchronous (false) and synchronous (true) camera stream transferring.

`tune_streaming_threads` (`bool`, `default: false`)

Enable/disable transfer tuning (buffering, timeouts, thread affinity). This tunes the streaming threads, such as assigning specific CPUs to threads based on affinity.

`turbo_mode` (`bool`, `default: false`)

Enable/disable Turbo Mode (if available) to push past the gigabit Ethernet speed ceiling, speeding up line and frame rates beyond the nominal link capacity.



**Topics**

`dalsa_camera` (`sensor_msgs/Image`)

RGB Image. Raw and compressed.

`dalsa_camera_720p` (`sensor_msgs/Image`)

Downscale RGB image to 720p. Raw and compressed.

`dalsa_camera_mono` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and compressed. Only published when `publish_mono` is `true`.

`dalsa_camera_mono_720p` (`sensor_msgs/Image`)

Mono (8UC1) Image, downscaled to 720p. Raw and compressed. Only published when `publish_mono` is `true`.

`ndvi` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed

Extra indexes that can be toggled on camera.yaml

`cvi` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed.  Only published when `CVI` is `true`.

`red_channel` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `Red` is `true`.

`green_channel` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `Green` is `true`.

`nir_channel` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `NIR` is `true`.

`normalized_green` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `Normalized_Green` is `true`.

`normalized_nir` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `Normalized_NIR` is `true`.

`normalized_red` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `Normalized_Red` is `true`.

`tvi` (`sensor_msgs/Image`)

Mono (8UC1) Image. Raw and Compressed. Only published when `TVI` is `true`.


**Node**

```
roslaunch dalsa_genie_nano_c2420 dalsa_genie_nano_c2420.launch
```

You can change the parameters on the launch file to suit your setup.
