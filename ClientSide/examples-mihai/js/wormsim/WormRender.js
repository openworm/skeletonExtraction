

var container, stats;

var camera, scene, objects, controls;
var particleLight;

// custom variables needed
var matrixValues = new Array();
var filesLoaded = false;
var moveWorm = false;


var OW_START_TIMESTEP = 65;
var timestep = OW_START_TIMESTEP;

var OW_NUM_OF_COLLADA_FILES = 5;

var loadedColladas = new Array();
var colladaLoadIte = 0;

function initRendering() {
    container = document.createElement('div');
    document.body.appendChild(container);

    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 1000);

    camera.position.x = 1 * 10 + 20;
    camera.position.y = 14;
    camera.position.z = 0 * 10 + 3;

    scene = new THREE.Scene();

    // Grid
    var size = 14, step = 1;
    var geometry = new THREE.Geometry();
    var material = new THREE.LineBasicMaterial({color: 0x303030});
    for (var i = -size; i <= size; i += step) {
        geometry.vertices.push(new THREE.Vector3(-size, -0.04, i));
        geometry.vertices.push(new THREE.Vector3(size, -0.04, i));
        geometry.vertices.push(new THREE.Vector3(i, -0.04, -size));
        geometry.vertices.push(new THREE.Vector3(i, -0.04, size));
    }
    var line = new THREE.Line(geometry, material, THREE.LinePieces);
    scene.add(line);

    // Lights
    particleLight = new THREE.Mesh(new THREE.SphereGeometry(4, 8, 8), new THREE.MeshBasicMaterial({color: 0xffffff}));
    scene.add(particleLight);
    scene.add(new THREE.AmbientLight(0xcccccc));
    var directionalLight = new THREE.DirectionalLight(/*Math.random() * 0xffffff*/0xeeeeee);
    directionalLight.position.x = Math.random() - 0.5;
    directionalLight.position.y = Math.random() - 0.5;
    directionalLight.position.z = Math.random() - 0.5;
    directionalLight.position.normalize();
    scene.add(directionalLight);
    var pointLight = new THREE.PointLight(0xffffff, 4);
    particleLight.add(pointLight);

    

    container.appendChild(renderer.domElement);

    stats = new Stats();
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.top = '0px';

    container.appendChild(stats.domElement);
    window.addEventListener('resize', onWindowResize, false);
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    //requestAnimationFrame(animate);
    render();
    stats.update();
}

// here, the timestep is calculated
function render() {
    var timer = Date.now() * 0.00000005; // 0.0005

    camera.lookAt(scene.position);

    particleLight.position.x = Math.sin(timer * 4) * 3009;
    particleLight.position.y = Math.cos(timer * 5) * 4000;
    particleLight.position.z = Math.cos(timer * 4) * 3009;

    THREE.AnimationHandler.update(timestep);
    //renderer.setTimestep(timestep);
    if (filesLoaded) {
        checkTimestepOverflow();
        // here the current skinning matrices are set to renderer !!! this is themost important line
        renderer.setCurrentMatrix(getCurrentMatrix());
        //hijackBoneMatrices();
    }

    renderer.render(scene, camera);
}

function hijackBoneMatrices() {
    console.log("hijackBoneMatrices -----------------------------");
    for (var i = 0; i < scene.children.length; i++) {
        //console.log(scene.children[i]);
        if (scene.children[i] instanceof THREE.Scene) {
            for (var j = 0; j < scene.children[i].children.length; j++) {
                //console.log(scene.children[i].children[j]);
                if (scene.children[i].children[j] instanceof THREE.Object3D) {
                    for (var k = 0; k < scene.children[i].children[j].length; k++) {
                        console.log(scene.children[i].children[j].children[k]);
                        if (scene.children[i].children[j].children[k] instanceof THREE.SkinnedMesh) {
                            console.log(scene.children[i].children[j].children[k]);
                            scene.children[i].children[j].children[k] = modifyObjectSkeleton(scene.children[i].children[j].children[k]);
                            console.log(scene.children[i].children[j].children[k]);
                        }
                    }
                }
            }
        }
    }

}

var expectedTypes = new Array();
expectedTypes[0] = 'Scene';
expectedTypes[1] = 'Object3D';
expectedTypes[2] = 'SkinnedMesh';
expectedTypes[3] = 'Bone';

function recurseIntoChildrenWithTimeOutput() {
    var start = new Date().getTime();
    recurseIntoChildren(scene.children, 0, 7);
    var end = new Date().getTime();
    var time = end - start;
    //console.log('Execution time: ' + time);
}

function recurseIntoChildren(childrenArray, currentDepth, depthMax) {
    for (childElementKey in childrenArray) {
        var childElement = childrenArray[childElementKey];
        var shouldIRecurse = false;
        if (childElement != undefined) {

            if (childElementKey == 'children')
                shouldIRecurse = true;
            else
            {    //if(childElement!= undefined) console.log(childElementKey + " " +childElement +" "+ childElement.constructor +" "+ childElementKey);  
                for (var i = 0; i < expectedTypes.length; i++)
                {
                    if (childElement.type != undefined)
                        if (childElement.type == (expectedTypes[i]))
                    {
                            shouldIRecurse = true;
                            if(childElement.type == expectedTypes[2]) {
                                modifyObjectSkeleton(childElement);
                            }
                            //console.log("Recursing because the object is of the given type...");
                            //console.log(currentDepth+" currentDepth");
                            //console.log(childElementKey);
                    }
                }
            }
            //if(shouldIRecurse) console.log(childElementKey + " :  shouldIRecurse: " + shouldIRecurse);

            if (shouldIRecurse && currentDepth < depthMax) {
                var newCurrentDepth = currentDepth+1;
                recurseIntoChildren(childElement, newCurrentDepth, depthMax);
            }
        }
    }
}

function modifyObjectSkeleton(object) {
    if (object.skeleton && object.skeleton.boneMatrices) {
            var currentMatrix = getCurrentMatrix();
            var modelMatrix = new Array();

            for (var n = 0; n < 16; n++) {
                modelMatrix[n] = object.skeleton.boneMatrices[n];
            }

            var newMatrix = new Array();

            if (currentMatrix.length == 0) {
                newMatrix = modelMatrix;
            } else {
                var mModel = new THREE.Matrix4();
                mModel.set(modelMatrix[0], modelMatrix[4], modelMatrix[8], modelMatrix[12],
                        modelMatrix[1], modelMatrix[5], modelMatrix[9], modelMatrix[13],
                        modelMatrix[2], modelMatrix[6], modelMatrix[10], modelMatrix[14],
                        modelMatrix[3], modelMatrix[7], modelMatrix[11], modelMatrix[15]);
                for (var m = 0; m < OW_NUM_NODES; m++) {
                    var mCurrent = new THREE.Matrix4();
                    mCurrent.set(currentMatrix[m * 16 + 0], currentMatrix[m * 16 + 4], currentMatrix[m * 16 + 8], currentMatrix[m * 16 + 12],
                            currentMatrix[m * 16 + 1], currentMatrix[m * 16 + 5], currentMatrix[m * 16 + 9], currentMatrix[m * 16 + 13],
                            currentMatrix[m * 16 + 2], currentMatrix[m * 16 + 6], currentMatrix[m * 16 + 10], currentMatrix[m * 16 + 14],
                            currentMatrix[m * 16 + 3], currentMatrix[m * 16 + 7], currentMatrix[m * 16 + 11], currentMatrix[m * 16 + 15]);

                    var mNew = new THREE.Matrix4();

                    mNew.multiplyMatrices(mModel, mCurrent);
                    for (var n = 0; n < 16; n++) {
                        newMatrix.push(mNew.elements[n]);
                    }
                }
            }
        
        object.skeleton.boneMatrices = newMatrix;
    }
    return object;
}

function getCurrentMatrix() {
    var currentMatrix = new Array();

    for (var m = 0; m < OW_NUM_NODES; m++) {
        var mNode = new THREE.Matrix4();
        var nodeMatrix = new THREE.Matrix4();
        mNode.set(matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 0], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 4], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 8], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 12],
                matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 1], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 5], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 9], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 13],
                matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 2], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 6], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 10], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 14],
                matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 3], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 7], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 11], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 15]);

        for (var n = 0; n < 16; n++) {
            currentMatrix.push(mNode.elements[n]);
        }
    }

    return currentMatrix;
}

function checkTimestepOverflow() {
    if (timestep > OW_END_TIMESTEP) {
        timestep = OW_START_TIMESTEP;
    }
    showTimestep(timestep);
}

function appendSceneToScreen() {
    document.getElementById('info2').innerHTML = JSON.stringify(scene);
}