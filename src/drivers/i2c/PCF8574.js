export function PCF8574(dev) { return {
    writer: dev.byteWriter,
    allocated: 0,
    preset: 0xFF,
    nextBlock: function(count=1) {
        if(this.allocated + count > 8) throw new "bits exhausted";
        var start = this.allocated;
        var mask = (Math.pow(2, count)-1) << start;
        this.allocated += count;
        console.log("alloc", mask);

        return function(data) {
            this.preset = (this.preset & ~mask) | ((dat & mask)<<start);
        };
    }
}
}
