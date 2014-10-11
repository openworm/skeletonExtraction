#ifndef m_mathH
#define m_mathH
//---------------------------------------------------------------------------
struct ShaderOptions {
	bool enabledTextures;
	bool useModelColor;
	bool doSkinning;
	bool useLighting;
	bool doNormalMapping;
	bool doColoring;
	bool showVertexSpaceAxis;
	bool doParallaxMapping;
	bool doDisplacementMapping;
	bool doBumpMapping;
	float bumpScale;
	float bumpMapSizeX;
	float bumpMapSizeY;
	float displacementScale;
	int g_ViewMode;
	float * fColor;
	int normalMapId;
	int heightMapId;
	int displacementMapId;
	int selectedSkeletonNode;
	bool doSTMMapping;
	bool doSDMMapping;
	bool doTSTXShifting;
	int operatorMapId;
	bool showOperatorMap;
	//bool recreateOperatorMap;
	bool doShowNormals;

	int normalsDMFilteringArea;
	int normalsDMFilteringIterations;
	int normalsDMFilteringDelta;
	int sdmMappingSpace;

	bool useTessellation;
	float tessLevelInner;
	float tessLevelOuter;

	int STMId;
	int SDMId;
	int SDMRId;
	bool showSDM;
	bool recreateSDM;
	bool showSTM;
	bool recreateSTM;
	bool doSTMSeamRender;

	ShaderOptions(){
		normalMapId = -1;
		heightMapId = -1;
		selectedSkeletonNode = -1;
		displacementMapId = -1;
		showOperatorMap = false;
		//recreateOperatorMap = false;
		showSDM = false;
		recreateSDM = false;
		showSTM = false;
		recreateSTM = false;
		doSTMMapping = false;
		doSDMMapping = false;

		normalsDMFilteringArea = 2;
		normalsDMFilteringIterations = 4;
		normalsDMFilteringDelta = 2;
		sdmMappingSpace = 1;
	}
};
//---------------------------------------------------------------------------
#endif
