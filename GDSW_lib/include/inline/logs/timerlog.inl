//---------------------------------------------------------------------------

inline Timerlog::Timerlog(char* name){
	timerName = name;
}
inline Timerlog::~Timerlog(){
}
inline void Timerlog::addStart(char* name){
	TimePrec now;
	now.milisecs = clock();
	now.start = true;
	now.name = name;

	times.push_back(now);
}
inline void Timerlog::addEnd(){
	TimePrec now;
	now.milisecs = clock();
	now.start = false;
	now.name = "";

	times.push_back(now);
}

inline void Timerlog::logExecutionTimes(){

	int num = times.size() / 2;

	for (int i=0; i < num; i++){
		TimePrec start = times.front();
		times.pop_front();
		TimePrec end = times.front();
		times.pop_front();

		double secDiff = (double)(end.milisecs - start.milisecs) / (double)CLOCKS_PER_SEC;
		int milisecDiff = (int)(secDiff * 1000) % 1000;

		char t[255];

		if (secDiff >= 1.0){
			sprintf ( t, "Timer(%s) :: %s :: %i sec and %02i milisec", 
				timerName,
				start.name,
				(int)secDiff,
				milisecDiff
				);
		} else {
			sprintf ( t, "Timer(%s) :: %s :: %02i milisec", 
				timerName,
				start.name,
				milisecDiff
				);
		}

		logger.logTex(string(t));
	}
}



#pragma package(smart_init)
