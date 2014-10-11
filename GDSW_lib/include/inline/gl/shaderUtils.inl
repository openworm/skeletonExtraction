//---------------------------------------------------------------------------
inline unsigned long getFileLength(std::ifstream& file)
{
	if(!file.good()) return 0;

	file.seekg(0,std::ios::end);
	unsigned long len = file.tellg();
	file.seekg(std::ios::beg);

	return len;
}

//---------------------------------------------------------------------------
inline void PrintShaderlog(const GLhandleARB obj_in)
{
	int infoLogLength = 0;
	int charsWritten  = 0;
	char *infoLog;

	glGetObjectParameterivARB(obj_in, GL_OBJECT_INFO_LOG_LENGTH_ARB, &infoLogLength);

	if (infoLogLength > 1)
		#ifdef _LOG
			logg.log(LOG_LEVEL_NOTE, "Shader Manager Log ::");
		#endif
	{
		infoLog = (char *)malloc(infoLogLength);
		if(!infoLog)
		{
			#ifdef _LOG
				logg.log(LOG_LEVEL_NOTE, "Allocation memory failed.\n");
			#endif
			return;
		}
		glGetInfoLogARB(obj_in, infoLogLength, &charsWritten, infoLog);
		#ifdef _LOG
			logg.log(LOG_LEVEL_NOTE, infoLog);
		#endif

		free(infoLog);
	}
}
//---------------------------------------------------------------------------
inline GLubyte* loadShaderSource(char* filename)
{
	GLubyte* ShaderSource;
	std::ifstream file;
	file.open(filename, std::ios::in);
	if(!file) return NULL;

	unsigned long len = getFileLength(file);

	if (len==0) return NULL;

	ShaderSource = (GLubyte*) new GLubyte[len+1];
	ShaderSource[len] = 0;

	unsigned int i=0;
	std::vector<int> indices;
	while (file.good())
	{
		ShaderSource[i] = file.get();
		if (ShaderSource[i] == '#')
			indices.push_back(i);

		if (!file.eof())
			i++;
	}

	ShaderSource[i] = 0;  // 0 terminate it.

	std::string str((const char*)ShaderSource);

	for (int j = 0; j < (int)(indices.size() / 2); j++) {
		std::string stringValue = std::to_string(getVariableIntValue(str.substr(indices[2 * j], (indices[2 * j + 1] - indices[2 * j] + 1))));
		str.replace(indices[2 * j], (indices[2 * j + 1] - indices[2 * j] + 1), (char*)(void*)stringValue.c_str());
	}

	file.close();

	GLubyte* ret = (GLubyte*)new GLubyte[str.length() + 1];
	strcpy((char*)ret,str.c_str());

	delete[] ShaderSource;
	ShaderSource = NULL;

	return ret;
}//---------------------------------------------------------------------------
inline int getVariableIntValue(std::string key){
	if(key == "#MAX_BONE_MAT#")
		return 24;
	if(key == "#NUM_OF_CTRL_BONES#")
		return 4;
	return -1;
}
