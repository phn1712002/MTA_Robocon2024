clear
# Inport Port
echo "Port?"
read PORT_VALUE

clear
docker images 
echo ""
echo "Name Image Docker?"
read NAME_IMAGE

# Start docker
ENV_NAME="$(basename $PWD)"
docker run -it \
	-p $PORT_VALUE:80 \
	--security-opt seccomp=unconfined \
	--shm-size=512m \
	-v=.:/root/$ENV_NAME \
	--device=/dev \
	--privileged \
	--name $ENV_NAME \
	$NAME_IMAGE
