Import("env")
import subprocess
MKFSTOOL=env.get("PROJECT_DIR") + '/tools/mklittlefs.' + subprocess.run(["uname", "-sm"],stdout=subprocess.PIPE).stdout.decode().replace(' ','.').rstrip();
env.Replace(MKFSTOOL=MKFSTOOL);
print("Using: "+MKFSTOOL);
#
# PlatformIO now believes it has actually created a SPIFFS
#

