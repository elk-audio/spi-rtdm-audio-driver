# SPI RTDM Audio Driver

Xenomai real-time audio driver for Audio over SPI for Elk Stereo Pi hat.

## Building

Have the kernel sources and an ARMv7 cross-compilation toolchain on your host machine, then do:

(https://github.com/elk-audio/elkpi-sdk) available on the host machine, then:

```
$ export KERNEL_PATH=<path to kernel source tree>
$ export CROSS_COMPILE=<arm compiler prefix>
$ make
```

It's possible to just use the official [Elk Audio OS cross-compiling SDK](https://github.com/elk-audio/elkpi-sdk), in which case you don't have to set the `CROSS_COMPILE` environment variable since the SDK will do it automatically.

As an alternative, you can build using the devshell option of Bitbake if you have all the Yocto layers ready on the host machine:

```
$ bitbake -c devshell virtual/kernel
$ export KERNEL_PATH=<path to kernel source in your bitbake tmp build files>
$ make
```


If the modules are installed already as part of the Kernel you can just do instead:

```
 $ modprobe spi_audio_rtdm audio_buffer_size=<BUFFER SIZE>
```

---
Copyright 2021 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
