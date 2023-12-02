
To flash FS:

- Place files at `data` dir
- Download and extract to this project root a **mklittlefs** executable for your OS [from a zipped binary here](https://github.com/earlephilhower/mklittlefs/releases) 
- Run PlatformIO project task: **Upload Filesystem Image**  (IMPORTANT!!!! Switch esp to upload mode and close ALL serial monitor windows)
- Rover will list files in FS on startup