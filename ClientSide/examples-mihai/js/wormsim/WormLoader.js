

    var NO_OF_COLLADA_FILES = 97;

    function loadAssets() {
        for (var i = 0; i < NO_OF_COLLADA_FILES; i++) {
            var timeoutPerioud = 1000 * i;
            var padi = padNumber(i, 1);
            var assetURL = "";
            if (i == NO_OF_COLLADA_FILES - 1) {
                assetURL = PATH_PREFIX+'openworm-example/res/animatedColladas/cuticle_anim_oneGeo.dae'
            } else {
                assetURL = PATH_PREFIX+'openworm-example/res/animatedColladas/muscles/muscle_anim' + i + '.dae';
            }
            loadAsset(assetURL);
            //setTimeout(function() {  },timeoutPerioud);
        }
    }

    function loadAsset(assetURL) {
        console.log("Load asset called");
        var loader = new THREE.ColladaLoader();
        loader.load(assetURL, colladaLoaded);
    }

// callback function to load the collada model
    function colladaLoaded(collada) {
        displayAndStoreCollada(collada);
        colladaLoadIte++;
    }

    function displayAndStoreCollada(collada) {
        var dae = collada.scene;
        /*
        dae.traverse(function (child) {
            if (child instanceof THREE.SkinnedMesh) {
                var animation = new THREE.Animation(child, child.geometry.animation);
                animation.play();
            }
        });
        */
        dae.scale.x = dae.scale.y = dae.scale.z = 0.026;
        dae.position.x = 15;
        dae.position.y = 8;
        dae.position.z = 9;

        dae.rotation.set(-Math.PI * 0.5, 0, 0);
        dae.updateMatrix();

        // Add the COLLADA
        scene.add(dae);
        loadedColladas[colladaLoadIte] = collada;
        if (colladaLoadIte == NO_OF_COLLADA_FILES - 1) {
            console.log("done loading colladas");
        }
    }
    
 