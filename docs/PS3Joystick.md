if you bind to you pc bt, switch it off while operating 

# Pairing for linux
Get self mac: `hcitool dev`
Recent ubuntu version (after 20.04) seems to bind sixpair automatically to bt mac. 
If it's not your case or you want to program other predefined mac instead.

0. make `sixpair` in `tools`
1. connect USB 
2. press and hold PS3 button
3. while holding button run `sudo ./sixpair 01:23:45:67:89:AB`
4. disconnect USB
5. press PS3 button once
6. device should be available through Bluetooth


# Pairing mac

Get self mac: `system_profiler SPBluetoothDataType`
0. cmake `sixaxispaiere` in tools 
1. press 


