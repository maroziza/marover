export default function(screen, lines = 8) {
    var page=0, position=0, lines;
    return {
        gotoPage: function(a,b) {
            page = b;
            screen.gotoPage(a,b);
        },
        drawLetters: function(a,b,c,d) {
            if(page<position) return;
            if(page>=(position+lines)) return;
            screen.drawLetters(a,b,c,d);
        },
        position: function(a) {
            // todo maybe clear
            position = Math.floor(a/8);
            screen.position(a);
        },
        from: function() { return position;},
        to: function() { return position+lines;}
    };
}
