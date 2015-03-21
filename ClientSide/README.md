###DESCRIPTION OF CLIENT-SIDE CODE:###

What changes have to be done in the webgl_loader_collada.html

1) added variables

			var quaternionValues = new Array();
			var matrixValues = new Array();
			var skeletonNodeValues = new Array();
			var filesLoaded = false;
			var moveWorm = false;
			
			var OW_START_TIMESTEP = 65;
			var OW_END_TIMESTEP = 6382;
			var OW_NUM_NODES = 31;
			
			var timestep = OW_START_TIMESTEP;

2) in one for loop load muscles and cuticles, always with new instance of the loader

			for (var i=0; i < 97; i++){                            
                var loader = new THREE.ColladaLoader();
                loader.options.convertUpAxis = true;			
				var padi = padNumber(i, 1);
								
					if (i == 96){
						// as a last model load cuticle
						loader.load( '../../SkinningData/Dropbox/animatedColladas/cuticle_anim_oneGeo.dae', loadCollada);
					} else {
						var url = '../../SkinningData/Dropbox/animatedColladas/muscles/muscle_anim'+i+'.dae';
                        console.log(url);
                        loader.load(url , loadCollada);					
					}
			}

3) new methods

	function loadCollada ( collada )
	function parseTransformationElem(n, elem)
	function parseTransformationData(data) 
	function readTransformationFile(file, ite)
	function parseSkeletonNodeElem(n, elem)
	function parseSkeletonNodeData(data)
	function readSkeletonFile(file)
	function padNumber(number, places)
	function readAllStreamedFiles()
	function performPreprocessing(i)

4) changed methods

function init()

				var textLoad = '<div>' + 'Loading transformation matrices... ' + '</div>';
				document.getElementById('info2').innerHTML = textLoad;
				
				$( "#moveworm" ).click(function() {
					if (moveWorm){
						moveWorm = false;
						$( "#moveworm" ).text("move worm");
					} else {
						moveWorm = true;
						$( "#moveworm" ).text("stop moving worm");					
					}
				});
			
				readAllStreamedFiles();
				
				
function render() 				
				
				if (filesLoaded){
					timestep = Math.round(timestep + clock.getDelta() * 11000);
					if (timestep > OW_END_TIMESTEP){
						timestep = timestep - (OW_END_TIMESTEP - OW_START_TIMESTEP);
					}
					performPreprocessing(timestep);
				}

				if (filesLoaded){
				
					var currentMatrix = new Array();
					
					var wormMovement = new THREE.Vector3(0, 0, 0);
					if (moveWorm){
						wormMovement = new THREE.Vector3(0, -200 + timestep / 10.0, 0);
					}
					
					// multiply matrices recursively in skeleton tree
					var fathersAffine = new THREE.Matrix4();
					
					
					for (var m = 0; m < OW_NUM_NODES; m++){
						
						var mNode = new THREE.Matrix4();
						var nodeMatrix = new THREE.Matrix4();	
						
						
						// this code is for recursive multiplication of matrices in skeleton chain...
						// if the original matrices are loaded, it has to be called
						// if postprocessed, it is not needed
						
						/*if (m > 0){
							 mNode.set(matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 0], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 1], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 2], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 3],
									   matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 4], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 5], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 6], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 7],
									   matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 8], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 9], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 10], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 11],
									   matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 12], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 13], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 14], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 15]);
						
							var translateToOriginMatrix = new THREE.Matrix4();
							translateToOriginMatrix.makeTranslation(-skeletonNodeValues[(m - 1) * 3], -skeletonNodeValues[(m - 1) * 3 + 1], -skeletonNodeValues[(m - 1) * 3 + 2]);
							
							var translateFromOriginMatrix = new THREE.Matrix4();
							translateFromOriginMatrix.makeTranslation(skeletonNodeValues[(m - 1) * 3], skeletonNodeValues[(m - 1) * 3 + 1], skeletonNodeValues[(m - 1) * 3 + 2]);

							// apply transformation of father to this childs matrix
							var mPart = new THREE.Matrix4();	
							mPart.multiplyMatrices(mNode, translateToOriginMatrix);	
							nodeMatrix.multiplyMatrices(translateFromOriginMatrix, mPart);	
						} else {
							nodeMatrix.makeTranslation(wormMovement.x, wormMovement.y, wormMovement.z);
						}
						
						var mRecursive = new THREE.Matrix4();	
						mRecursive.multiplyMatrices(fathersAffine, nodeMatrix);						
					
						for (var n = 0; n < 16; n++){
							currentMatrix.push(mRecursive.elements[n]);
						}
						
						fathersAffine = mRecursive;*/
						
						
						 mNode.set(matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 0], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 4], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 8], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 12],
								   matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 1], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 5], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 9], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 13],
								   matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 2], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 6], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 10], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 14],
								   matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 3], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 7], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 11], matrixValues[timestep * OW_NUM_NODES * 16 + m * 16 + 15]);
						
						for (var n = 0; n < 16; n++){
							currentMatrix.push(mNode.elements[n]);
						}
						
					}
					
					renderer.setCurrentMatrix(currentMatrix);
				}

				
What changes have to be done in the tree.js

1) added variables and methods

	var timestep;
	var currentMatrix = new Array();
	
	this.setTimestep = function setTimestep(ts) {
		timestep = ts;
	}
	
	this.setCurrentMatrix = function setCurrentMatrix(m) {
		currentMatrix = m;
	}

2) changed methods

	injection is performed where the code _gl.uniformMatrix4fv( p_uniforms.boneGlobalMatrices, false, object.skeleton.boneMatrices ) is called
	
					var modelMatrix = new Array();
					
					for (var n = 0; n < 16; n++){
						modelMatrix[n] = object.skeleton.boneMatrices[n];
					}
				
					var newMatrix = new Array();
					
					if (currentMatrix.length == 0){
						newMatrix = modelMatrix;
					} else {
						var mModel = new THREE.Matrix4();
						mModel.set(modelMatrix[0], modelMatrix[4], modelMatrix[8], modelMatrix[12],
								   modelMatrix[1], modelMatrix[5], modelMatrix[9], modelMatrix[13],
								   modelMatrix[2], modelMatrix[6], modelMatrix[10], modelMatrix[14],
								   modelMatrix[3], modelMatrix[7], modelMatrix[11], modelMatrix[15]);
						for (var m = 0; m < OW_NUM_NODES; m++){
							var mCurrent = new THREE.Matrix4();
							mCurrent.set(currentMatrix[m * 16 + 0], currentMatrix[m * 16 + 4], currentMatrix[m * 16 + 8], currentMatrix[m * 16 + 12],
							   		     currentMatrix[m * 16 + 1], currentMatrix[m * 16 + 5], currentMatrix[m * 16 + 9], currentMatrix[m * 16 + 13],
									     currentMatrix[m * 16 + 2], currentMatrix[m * 16 + 6], currentMatrix[m * 16 + 10], currentMatrix[m * 16 + 14],
									     currentMatrix[m * 16 + 3], currentMatrix[m * 16 + 7], currentMatrix[m * 16 + 11], currentMatrix[m * 16 + 15]);
													   
							var mNew = new THREE.Matrix4();	

							mNew.multiplyMatrices(mModel, mCurrent);
							for (var n = 0; n < 16; n++){
								newMatrix.push(mNew.elements[n]);
							}
						}
					}
					
					_gl.uniformMatrix4fv( p_uniforms.boneGlobalMatrices, false, newMatrix );

###HTML FILES:###

webgl_loader_collada_openworm.html - first version of skinned cuticle
webgl_loader_collada_openworm_anchored.html - the cuticle is anchored in the middle
webgl_loader_collada_openworm_anchored2.html - the cuticle is anchored in 1/4 from the head
webgl_loader_collada_openworm_changedInjection.html - tryouts to change tree.js injection of skinning matrices
webgl_loader_collada_openworm_musclesanchored2.html - muscles anchored in 1/4 from head
webgl_loader_collada_openworm_post.html - postprocessed rotations, with SLERP filtering radius 100
webgl_loader_collada_openworm_post2.html - postprocessed rotations, with SLERP filtering radius 200
webgl_loader_collada_openworm_postArmy.html - more anymated cuticles 
webgl_loader_collada_openworm_reindexedColladas.html - muscles models with diffrent IDs in the geoemtry tag
indexMihai.html - html from mihai, it can load static muscles
index.html - animated cuticle and muscles, with rotatios anchored in the middle of the cuticle
index2.html - animated cuticle and muscles, with rotatios anchored in 1/4 from the head
index2_downsampled10.html - animated cuticle and muscles, with rotatios anchored in 1/4 from the head, downsampled, only every 10th rotation is used
index2_downsampled25.html - animated cuticle and muscles, with rotatios anchored in 1/4 from the head, downsampled, only every 25th rotation is used
index2_downsampled50.html - animated cuticle and muscles, with rotatios anchored in 1/4 from the head, downsampled, only every 50th rotation is used


###COLLADA FILES:###

DropBox/animatedColladas/animation_wormMuscleCollada_00000.dae - animated muscles in one geometry with old skinning weights
DropBox/animatedColladas/animation_wormMeshAndSkinningData_31S_0ite.dae - animated cuticle with all skinning weights
(*) DropBox/animatedColladas/cuticle_anim_oneGeo.dae - animated cuticle in one collada geoemtry object
DropBox/animatedColladas/muscles_anim_partialGeo.dae - all animated muscles in one collada file, each muscle in separate geoemtry object
DropBox/animatedColladas/muscles_anim_oneGeo.dae - all animated muscles in one collada file, all muscles in one object
DropBox/animatedColladas/eveything_anim_partialGeo.dae - animated cuticle and muscles. everything in separate geometry
(*) DropBox/animatedColladas/muscles/* - animated muscles, one muscle per one colada file

DropBox/staticColladas/wormMuscleColladaMuscleCellsAll_00000.dae - static muscles in separate geometry
DropBox/staticColladas/wormMuscleCollada_00000.dae - static muscles in one geometry
DropBox/staticColladas/wormMeshAndSkinningData_31S_0ite.dae - static cuticle with old skinning weights
DropBox/staticColladas/wormColladaCuticleAndAllMuscleCells_00000.dae - static cuticle and muscles with old skinning weights in separate geometry


###TRANSFORMATION FILES:###

DropBox/originalQuaternions - quaternions exported from skeleton differences
DropBox/originalMatrices - matrices exported from skeleton differences
DropBox/postprocessed - transformations are multiplied recursively in the skeleton chain
DropBox/postprocessed_100 -  - postprocessed matrices that are filtered out and smoothed by SLERP interpolation with radius 100
DropBox/postprocessed_400 -  - postprocessed matrices that are filtered out and smoothed by SLERP interpolation with radius 400
DropBox/anchored - rotations anchored in the middle of the cuticle and postprocessed by SLERP radius od 100
(*) DropBox/anchored2 - rotations anchored in 1/4 from head and postprocessed by SLERP radius od 100


