ninja -C build;
picotool reboot -uf;
sleep 2;
picotool load build/backscatter_beamforming.elf;
sleep 5;
picotool reboot;