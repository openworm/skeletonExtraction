//---------------------------------------------------------------------------

inline Log::Log(){
}

inline Log::~Log(){
}

inline void Log::log(int level, const char * text){
	if (level > logLevel)
		return;
	if (logTime){
		time_t rawtime;

		time ( &rawtime );
		char t[255];
		struct tm timeinfo;
		localtime_s (&timeinfo, &rawtime );

		int ticks = GetTickCount();
		int mili = ticks % 1000;

		sprintf_s ( t, sizeof(t), "%02i:%02i:%02i_%03i - (%01i) - ", 
			timeinfo.tm_hour,
			timeinfo.tm_min,
			timeinfo.tm_sec,
			mili,
			level
		);

		logger.logTex(std::string(t) + std::string(text));
	}
	else {
		logger.logTex(std::string(text));
	}
}

inline void Log::log(int level, const char * text, float f){
	if (level > logLevel)
		return;
	char strBuffer[255] = {0};
	sprintf_s(strBuffer, sizeof(strBuffer), (std::string(text) + std::string(" %16.16f")).c_str(), f);
	log(level, strBuffer);
}

inline void Log::log(int level, const char * text, int i){
	if (level > logLevel)
		return;
	char strBuffer[255] = {0};
	sprintf_s(strBuffer, sizeof(strBuffer),(std::string(text) + std::string(" %i")).c_str(), i);
	log(level, strBuffer);
}

inline void Log::log(int level, const char * text, BYTE i){
	if (level > logLevel)
		return;
	char strBuffer[255] = {0};
	sprintf_s(strBuffer, sizeof(strBuffer),(std::string(text) + std::string(" %i")).c_str(), (int)i);
	log(level, strBuffer);
}

inline void Log::log(int level, const char * text, CVector3 v){
	if (level > logLevel)
		return;
	char strBuffer[255] = {0};
	sprintf_s(strBuffer, sizeof(strBuffer),(std::string(text) + std::string(" (%f, %f, %f)")).c_str(), v.x, v.y, v.z);
	log(level, strBuffer);
}

inline void Log::log(int level,  const char * text, bool b){
	std::string s;
	if(b)
		s = "true";
	else
    	s = "false";
	log(level, (std::string(text) + " : " + s).c_str());
}

inline void Log::log(int level, float * arr, int length){
	if (level > logLevel)
		return;
	std::string s;
	for (int i = 0; i<length; i++){
		std::string f = std::to_string((float)arr[i]);
		s += f;
		s += " ";
	}
	log(level, s.c_str());
}

inline void Log::log(int level, TNT::Array1D< float > vector){
	if (level > logLevel)
		return;
	std::string row;
	for (int i = 0; i < vector.dim1(); i++) {
		std::string s = std::to_string(vector[i]);
		int l = intlength;
		if (s.length() < intlength)
			l = s.length();
		row += s;
		for (int j = 0; j < floatlength+1-l ; j++)
			row += " ";
	}
	log(level, row.c_str());
}
inline void Log::log(int level, vector< int > vector){
	if (level > logLevel)
		return;
	std::string row;
	for (int i = 0; i < vector.size(); i++) {
		std::string s = std::to_string(vector[i]);
		int l = intlength;
		if (s.length() < intlength)
			l = s.length();
		row += s;
		for (int j = 0; j < intlength+1-l ; j++)
			row += " ";
	}
	log(level, row.c_str());
}
inline void Log::log(int level, TNT::Array2D< float > matrix){
	if (level > logLevel)
		return;
	for (int i = 0; i < matrix.dim1(); i++) {
		std::string row;
		for (int j = 0; j < matrix.dim2(); j++) {
			std::string s = std::to_string(matrix[i][j]);
			int l = floatlength;
			if (s.length() < floatlength)
				l = s.length();
			row += s;
			for (int j = 0; j < floatlength+1-l ; j++)
				row += " ";
		}
		logger.logTex((char *)row.c_str());
	}
}

inline void Log::log(int level, TNT::Array2D< double > matrix){
	if (level > logLevel)
		return;
	for (int i = 0; i < matrix.dim1(); i++) {
		std::string row;
		for (int j = 0; j < matrix.dim2(); j++) {
			std::string s = std::to_string((float)matrix[i][j]);
			int l = floatlength;
			if (s.length() < floatlength)
				l = s.length();
			row += s;
			for (int j = 0; j < floatlength+1-l ; j++)
				row += " ";
		}
		logger.logTex((char *)row.c_str());
	}
}

inline void Log::logTransformation(int level, TNT::Array2D< float > matrix){
	if (level > logLevel)
		return;
	for (int i = 0; i < matrix.dim1(); i++) {
		string row;
		for (int j = 0; j < matrix.dim2(); j++) {
			std::string s = std::to_string(matrix[i][j]);
			int l = translength;
			if (s.length() < translength)
				l = s.length();
			row += s;
			for (int j = 0; j < translength+1-l ; j++)
				row += " ";
		}
		logger.logTex((char *)row.c_str());
	}
}

inline void Log::log(int level, TNT::Array2D< bool > matrix){
	if (level > logLevel)
		return;
	for (int i = 0; i < matrix.dim1(); i++) {
		string row;
		for (int j = 0; j < matrix.dim2(); j++) {
			string s;
			if(matrix[i][j])
				s = "X";
			else
    			s = "0";
			int l = boolength;
			if (s.length() < boolength)
				l = s.length();
			for (int j = 0; j < l; j++)
				row += s[j];
			for (int j = 0; j < boolength+1-l ; j++)
				row += " ";
		}
		log(level, row.c_str());
	}
}

inline void Log::logTransformationPointer(int level, float * matrix){
if (level > logLevel)
		return;
	for (int i = 0; i < 4; i++) {
		string row;
		for (int j = 0; j < 4; j++) {
			std::string s = std::to_string(matrix[i * 4 + j]);
			int l = translength;
			if (s.length() < translength)
				l = s.length();
			row += s;
			for (int j = 0; j < translength+1-l ; j++)
				row += " ";
		}
		log(level, row.c_str());
	}
}

#pragma package(smart_init)
