clear
# Remove
rm -rf build
rm -rf install
rm -rf log

# Create
mkdir .backup
touch ./.backup/.gitignore
echo '*' > ./.backup/.gitignore
touch ./.backup/COLCON_IGNORE

mkdir build
touch ./build/.gitignore
echo '*' > ./build/.gitignore

mkdir install
touch ./install/.gitignore
echo '*' > ./install/.gitignore

mkdir log
touch ./log/.gitignore
echo '*' > ./log/.gitignore

mkdir model
touch ./model/.gitignore
echo '*' > ./model/.gitignore
touch ./model/COLCON_IGNORE

mkdir src
touch ./src/.gitkeep

mkdir launch
touch ./launch/.gitkeep

mkdir firmware
touch ./firmware/.gitkeep

# Check ws
rosdep install -i --from-path src --rosdistro humble -y

# Build ws
colcon build

# Get pwd
DIR="$( cd "$( dirname "$0" )" && pwd )"
# Echo /home/ubuntu/.bashrc in user ubuntu
echo "" >> /home/ubuntu/.bashrc
echo "#### THIS IS OF [$DIR]" >> /home/ubuntu/.bashrc
echo "source $DIR/install/local_setup.bash" >> /home/ubuntu/.bashrc
echo "########################" >> /home/ubuntu/.bashrc
echo "" >> /home/ubuntu/.bashrc
echo "clear" >> /home/ubuntu/.bashrc


# Echo /root/.bashrc in user ubuntu
sudo echo "" >> /root/.bashrc
sudo echo "#### THIS IS OF [$DIR]" >> /root/.bashrc
sudo echo "source $DIR/install/local_setup.bash" >> /root/.bashrc
sudo echo "########################" >> /root/.bashrc
sudo echo "" >> /root/.bashrc
sudo echo "clear" >> /root/.bashrc

# Remove git
while true; do 
read -p "Remove .git? (y/n) " yn
case $yn in 
	[yY] ) echo "Remove .git";
		rm -rf .git
		rm -rf readme.md
		break;;
	[nN] ) echo "No remove .git";
		break;;
	* ) echo "Invalid Response";;
esac
done

# Echo Complete
clear
echo "Complete !"
