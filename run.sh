killall -9 /usr/local/bin/qjs
stty 115200 -icanon -F /dev/ttyUSB0
while true; do
    $1 &
    PI=$!
    echo ======================= `date` $PI;
    sleep 0.2;
    inotifywait -e modify -q `find  ../src . `;
    kill -9 $PI 2> /dev/null > /dev/null
    killall -9 /usr/local/bin/qjs
    wait $PI
    sleep 0.3;
done
