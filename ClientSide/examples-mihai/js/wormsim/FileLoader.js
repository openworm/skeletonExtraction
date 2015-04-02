// load muscles and the cuticle

//var skeletonNodeValues;

// rad all the local transformations			
function readAllStreamedFiles() {
    //console.log('Start loading files.');
    for (var i = OW_START_TIMESTEP; i <= OW_END_TIMESTEP; i += 1) {
        var padi = padNumber(i, 4);
        readTransformationFile(PATH_PREFIX+'openworm-example/res/anchored2/matrix_anchored_31S_' + padi + '.mat', i);
    }
}

function readTransformationFile(file, ite) {
    jQuery.ajax({
        url: file,
        success: parseTransformationData,
        async: false
    });

    if (ite == OW_END_TIMESTEP) {
        //console.log('All quaternion files loaded !');
        filesLoaded = true;
    }
}

function parseTransformationData(data) {
    var lines = data.split("\n");
    $.each(lines, parseTransformationElem);
}

// these methods read the rotations from text file
function parseTransformationElem(n, elem) {
    var qvalues = elem.split(" ");
    if (qvalues.length >= 4) {
        matrixValues.push(qvalues[0]);
        matrixValues.push(qvalues[1]);
        matrixValues.push(qvalues[2]);
        matrixValues.push(qvalues[3]);
    }
}
/*
function parseSkeletonNodeElem(n, elem) {
    var qvalues = elem.split(" ");
    if (qvalues.length >= 3) {
        skeletonNodeValues.push(qvalues[0]);
        skeletonNodeValues.push(qvalues[1]);
        skeletonNodeValues.push(qvalues[2]);
    }
}

function parseSkeletonNodeData(data) {
    var lines = data.split("\n");
    $.each(lines, parseSkeletonNodeElem);
}

// rad skeleton from text file, not needed anymore, because the tranformationsa re already postprocessed before client side
function readSkeletonFile(file) {
    jQuery.ajax({
        url: file,
        success: parseSkeletonNodeData,
        async: false
    });
}
*/
