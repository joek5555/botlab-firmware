cd /home/pi/mbot-firmware/build
cmake ..
make
echo "Enter path to desired script to upload with respect to mbot-firmware/build: "  
read file_path
cd /home/pi/mbot-firmware/build/$file_path

echo "Please put Pico in boot mode"
read -p "Press enter to continue"
echo "Enter name of script to upload with extension .uf2: "  
read script_name
sudo picotool load -f $script_name 
sudo picotool reboot

sudo pkill timesync
sudo pkill pico_shim

echo "Please unplug and plug in the Pico USB" 
read -p "Press enter to continue"

cd /home/pi/botlab-w-23/system_compilation/bin
./timesync &
sudo ./pico_shim &
