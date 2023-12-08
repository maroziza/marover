# Building under linux

Installing all dependencies. Note that
ubuntu has rather old platformio version, 
so install straight from pip

```
pip3 install platformio
pio platform install espressif32
# compile
pio run
# upload compiled firmware
pio run -t upload
# upload filesystem image (data dir)
pio run -t uploadfs
```