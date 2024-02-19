function TeeTap(proxy, printer) {
return function(data) {
    printer(data);
    proxy(data);
    return data;
}
}
