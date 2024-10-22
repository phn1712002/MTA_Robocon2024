# ROS2
sudo echo "" >> /root/.bashrc
sudo echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
sudo echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
sudo echo "" >> /root/.bashrc

#System
sudo apt update
sudo apt install python-is-python3 -y
sudo apt install python3-venv -y
sudo apt install nano -y
sudo apt install evtest -y
sudo apt install jstest-gtk -y

# Setting pio
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm -rf get-platformio.py

# Echo /home/ubuntu/.bashrc in user ubuntu
echo "" >> /home/ubuntu/.bashrc
echo "#### THIS IS OF PlatformIO" >> /home/ubuntu/.bashrc
echo "export PATH=\$PATH:/root/.platformio/penv/bin" >> /home/ubuntu/.bashrc
echo "########################" >> /home/ubuntu/.bashrc
echo "" >> /home/ubuntu/.bashrc

# Echo /root/.bashrc in user ubuntu
sudo echo "" >> /root/.bashrc
sudo echo "#### THIS IS OF PlatformIO" >> /root/.bashrc
sudo echo "export PATH=\$PATH:/root/.platformio/penv/bin" >> /root/.bashrc
sudo echo "########################" >> /root/.bashrc
sudo echo "" >> /root/.bashrc
