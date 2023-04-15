sudo pkill timesync
sudo pkill pico_shim

echo "Please unplug and plug in the Pico USB" 
read -p "Press enter to continue"

cd /home/pi/botlab-w-23/system_compilation/bin
./timesync &
sudo ./pico_shim &
echo "Press enter to finish"
