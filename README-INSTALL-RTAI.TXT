# Install base distribution
#  - We use Zenwalk 2.6 (kernel 2.6.16.16)
#  - Newer kernels no not yet have a stable RTAI patch

# Get the RTAI source
cd /root
cvs -d:pserver:anonymous@cvs.gna.org:/cvs/rtai co magma

# Get the kernel source
cd /usr/src
wget http://www.kernel.org/pub/linux/kernel/v2.6/linux-2.6.16.tar.bz2
tar -xvjf linux-2.6.16.tar.bz2
ln -s linux-2.6.16 linux
cd linux

# Patch the kernel with RTAI
patch -p1 < /root/magma/base/arch/i386/patches/hal-linux-2.6.16-i386-1.3-00.patch
make menuconfig
--> Local version, Append a kernel suffix (-ipipe)
--> Turn of ACPI (Adv Cfg and Pwr Interface), APM, and module versioning
--> If the processor does not support APIC (Adv Prog Int Controller), turn it off
make bzImage
make modules
make modules_install
cp arch/i386/boot/bzImage /boot/vmlinuz-2.6.16-ipipe
--> edit etc/lilo.conf
lilo
shutdown -r now
--> Select the adeos kernel

# Install RTAI
cd magma
make
make install
cp -a /dev/rtai_shm /lib/udev/devices/
cp -a /dev/rtf[0-9] /lib/udev/devices/
vi /etc/ld.so.conf <-- Add /usr/realtime/lib
ldconfig -n /usr/realtime/lib

# Insert the realtime kernel modules
# Append to /etc/rc.d/rc.modules
insmod /usr/realtime/modules/rtai_hal.ko
insmod /usr/realtime/modules/rtai_lxrt.ko
insmod /usr/realtime/modules/rtai_fifos.ko

# Test RTAI installation
cd /usr/realtime/testsuite/kern/latency/
./run

# Install subversion
cd /root
wget http://subversion.tigris.org/downloads/subversion-1.4.0.tar.bz2
wget http://subversion.tigris.org/downloads/subversion-deps-1.4.0.tar.bz2
tar -xvjf subversion*
cd subversion
./configure
make
make install

# Install CAN driver
Make sure BIOS setting "Plug & Play O/S" is turned off
tar -zxvf esdcan (3.7.0)
cd esdcan
vi config.mk <-- linuxrtai, pci266
make
cp src/esdcan-pci200.ko /lib/modules/`uname -r`/
cp lib32/libntcan.* /usr/local/lib
ldconfig -n /usr/local/lib
cd /lib/udev/devices
mknod --mode=a+rw can0 c 54 0
mknod --mode=a+rw can1 c 54 1


