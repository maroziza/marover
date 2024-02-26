export function PCF8574(dev) { return {
    reader: dev.byteReader,
    writer: dev.byteWriter,
    allocated: 0,
    preset: 0,
    nextBlock: function(count=1) {
        if(this.allocated + count > 8) throw new "bits exhausted";
        var start = this.allocated;
        var mask = (Math.pow(2, count)-1) << start;
        this.allocated += count;
        console.log("alloc", mask);
        var self = this;
        return function(data) {
            self.preset = (self.preset & ~mask) | ((data<<start) & mask);
            self.writer(~self.preset);
        };
    }
}
}
