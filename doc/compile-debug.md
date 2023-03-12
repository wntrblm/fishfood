## LumenPNP Session Sonnabend, 11.3.2023  
Peter, Mona, Ekki

### Ziele:  
* Alle können die Firmware (fishfood) compilieren (cross-dev toolchain)  
* Alle haben die Tools fürs Debugging installiert und getestet


### Vorgeschlagene Ordnerstruktur:

```
projektfolder/
    +-/pico/
    +-/pico/
        +-/pico-sdk/
    +-/fishfood/
    +-/openocd/
```


Wir folgenden dem Guide [Getting Started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)




## 1. Repos clonen (pico-sdk, fishfood)

```
cd <projektfolder>
cd pico/
git clone https://github.com/raspberrypi/pico-sdk.git --branch master
cd pico-sdk/
git submodule update --init
```

```
cd <projektfolder>
git clone https://github.com/casartar/fishfood.git
```

### 1.1 Pfad für Pico-sdk exportieren (permanent in .zshrc/.bashrc)
```
export PICO_SDK_PATH=~/<path_to_projektfolder>/pico/pico-sdk/
```



## 2. cmake und crossdev Tools installieren.
Der Kram ist bei Arch in den normalen Repos enthalten, nix AUR oder so. Manjaro?

* cmake
* arm-none-eabi-gcc
* arm-none-eabi-newlib
* arm-none-eabi-binutils (wird bei Arch automatisch mit installiert)
* arm-none-eabi-gdb



## 3. Firmware bauen
Erzeugt die binaries der firmware für jellyfish und starfish, sowie die ELF Dateien fürs debuggen.

```
cd <projektfolder>/fishfood/
mkdir build
cd build
cmake ..
make -j4
```


## 4. Debugging Tools

Verwendete Debug HW:
* picoprobe (RPi Pico mit Picoprobe-SW und passendem Kabel)
* Segger J-Link EDU Mini (funktioniert laut Peter etwas besser)


### 4.1 OpenOCD installieren
**OCD**: On-Chip-Debugger, stellt logische Verbindung zw. Embedded HW und GDB (oder ähnlichen Tools) her.
Wir benötigen eine spezielle Version von OpenOCD (0.11.0-g8e3c38f), zur Verfügung gestellt von Raspberry Pi, die auch den RP2040 unterstützt. Noch nicht upstream verfügbar.
OpenOCD wird mit autotools gebaut, nicht mit cmake.

```
cd <projektfolder>
git clone https://github.com/raspberrypi/openocd.git --branch rp2040 --recursive --depth=1
cd openocd
./bootstrap
./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio
make -j4
sudo make install
```

Der letzte Schritt ist nicht unbedingt nötig, falls man seine Distro-abhängige Installation von OpenOCD nicht zerschiessen will. Falls man das also nicht installiert, muss man die custom-OpenOCD Version mit Pfad aufrufen und auch später den kompletten Pfad zum executable in den Launch-Files für VScode angeben.

### 4.2 Probe anschliessen

### 4.3 openocd starten
Wenn alles klappt erzeugt das einen Socket, auf den GDB zugreifen kann.

### 4.4 Integration in VScode
Wir folgen der [Anleitung bei Digikey](https://www.digikey.de/de/maker/projects/raspberry-pi-pico-and-rp2040-cc-part-2-debugging-with-vs-code/470abc7efb07432b82c95f6f67f184c0)


