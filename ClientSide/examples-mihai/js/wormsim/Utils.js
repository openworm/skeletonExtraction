// pad numbers
function padNumber(number, places) {
    var result = "";
    if (number != 0) {
        while (Math.pow(10, places) > number) {
            result += "0";
            places--;
        }
    } else {
        for (var i = 0; i < places; i++) {
            result += "0";
        }
    }
    result += number;
    return result;
}

// postprocessing is not needed anymore, cause itis called on server side			
function showTimestep(i) {
    updateStatusText('Timestep: ' + i);
}

function updateStatusText(text) {
    document.getElementById('info2').innerHTML = text;
}


