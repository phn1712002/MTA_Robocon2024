#Camera
sudo usermod -aG video $USER
newgrp video

#Audio
sudo usermod -aG audio $USER
newgrp audio

#DialOut
sudo usermod -aG dialout $USER
newgrp dialout

# Edit card
cat /proc/asound/cards
sudo nano /usr/share/alsa/alsa.conf