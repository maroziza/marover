while true; do
    $1 &
    PI=$!
    echo ======================= `date` $PI;
    inotifywait -q `find  ../src . `;
    kill -9 $PI 2> /dev/null > /dev/null
    wait $PI
    sleep 0.2;
done
