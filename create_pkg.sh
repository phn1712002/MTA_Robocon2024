clear
# Create package
echo "Name of package?: "
read PKG_NAME
cd src
while true; do
read -p "C++ or Python, create package with C++ (y/n) ?: " yn
case $yn in 
	[yY] ) echo "Create package with C++";
		ros2 pkg create --build-type ament_cmake $PKG_NAME
		break;;
	[nN] ) echo "Create package with Python";
		ros2 pkg create --build-type ament_python $PKG_NAME
		break;;
	* ) echo "Invalid response";;
esac
done
