pxy2_firmware-3.0.20-general.hex


cd include

git clone https://github.com/charmedlabs/pixy2.git

cd pixy2/scripts && ./build_libpixyusb2.sh

roslaunch pixy2_ros pixy_camera.launch agent_name:=agent7

