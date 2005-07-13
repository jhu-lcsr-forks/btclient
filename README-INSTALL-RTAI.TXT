Software Installation Guide - Installation of RTAI-3.1 with Kernel 2.4.27: 

Get the necessary software tools:
# apt-get install lynx
# apt-get install bzip2
# apt-get install ncurses-dev
# apt-get install ssh
# apt-get install gcc
# apt-get install less
# apt-get install make

Unpack the kernel source: 
# cd /usr/src
# tar xvjf linux-2.4.27.tar.bz2
# ln -s linux-2.4.27 linux

Unpack RTAI to /usr/src/rtai-3.1 and 
# ln -s rtai-3.1 rtai

Patch the kernel: 
# cd /usr/src/linux
# patch -p1 < ../rtai/rtai-core/arch/i386/patches/hal16-2.4.27.patch

Copy the existing (debian) kernel config file to /usr/src/linux 
# cp /boot/config-2.4.xxx /usr/src/linux/.config

Configure the kernel: 
# make menuconfig
(you need to "make menuconfig" even if you accept the default config!) 

Make sure that 
	"Adeos" is selected
	"Loadable module support -> Set version information on all module symbols" is disabled 
	Select the processor type (>386)
	Deselect WAN support
	Deselect SMP support
	Select APIC for uniprocessor
    
Compile the kernel: 
# make dep
# make bzImage
# make modules
# make modules_install

Install the kernel: 
# cd /usr/src/linux
# cp arch/i386/boot/bzImage /boot/vmlinuz-2.4.27-adeos
# cp System.map /boot/System.map-2.4.27-adeos

Modify LILO or GRUB for the new kernel
# vi /etc/lilo.conf
# lilo

After the kernel is compiled and installed successfully, compile RTAI: 
# cd /usr/src/rtai
# make menuconfig
If you want to do some serial port I/O enable "Add-ons -> Serial line driver"
(if you're new to RTAI just accept the default config - after saving the config wait until all configuration is done) 
# make
# make install
If no errors occured, reboot now, select the new kernel from the boot menu. 

To test RTAI: 
# cd /usr/realtime/testsuite/kern/latency/
# ./run

Install the CAN driver
# tar -zxvf esdcan-xxx
# cd esdcan-xxx
# make
# cp esdcan-xxx /lib/modules/2.4.27-adeos/esdcan-driver
Add "can	esdcan-driver" to /etc/modutils/aliases
Add "can" to /etc/modules
# update-modules

Install the WAM software
# tar -zxvf btech_xxx


