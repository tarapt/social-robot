cd ARIA
sudo dpkg -i libaria_2.9.1+ubuntu16_amd64.deb
sudo dpkg -i libaria-python_2.9.1+ubuntu16_amd64.deb

cd ../BaseArnl
sudo dpkg -i arnl-base_1.9.2+ubuntu16_amd64.deb 
sudo dpkg -i arnl-base-python_1.9.2+ubuntu16_amd64.deb

cd ../ARNL
sudo dpkg -i libarnl_1.9.2+ubuntu16_amd64.deb 
sudo dpkg -i libarnl-python_1.9.2+ubuntu16_amd64.deb

cd ../SONARNL
sudo dpkg -i libsonarnl_1.9.2+ubuntu16_amd64.deb
sudo dpkg -i libsonarnl-python_1.9.2+ubuntu16_amd64.deb 

cd ../MobileSim
sudo dpkg -i mobilesim_0.9.8+ubuntu16_amd64.deb

cd ../MobileEyes
sudo dpkg -i mobileeyes_2.2.4-1_i386.deb
sudo -E apt --fix-broken install

cd ../Mapper3
sudo dpkg -i mapper3_2.2.5-1_i386.deb
sudo -E apt --fix-broken install