//---------------------------------------------------------------------------
#include "lbse_openCLManager.h"

int OpenCLManager::FloorPow2(int n)
{
	int exp;
	frexp((float)n, &exp);
	return 1 << (exp);
}

unsigned long OpenCLManager::getFileLength2(std::ifstream& file)
{
	if(!file.good()) return 0;

	file.seekg(0,std::ios::end);
	unsigned long len = file.tellg();
	file.seekg(std::ios::beg);

	return len;
}

char * OpenCLManager::loadKernelSource(char* filename)
{
	char* kernelSource;
	std::ifstream file;
	file.open(filename, std::ios::in);
	if(!file) return NULL;

	unsigned long len = getFileLength2(file);

	if (len==0) return NULL;

	kernelSource = (char*) new char[len+1];
	kernelSource[len] = 0;

	unsigned int i=0;
	std::vector<int> indices;
	while (file.good())
	{
		kernelSource[i] = file.get();
		if (kernelSource[i] == '#')
			indices.push_back(i);

		if (!file.eof())
			i++;
	}

	kernelSource[i] = 0;
	return kernelSource;
}

void OpenCLManager::openCL_LaplaceContraction(CVector3 * output, float* input, unsigned int numOfVertices, float * Lcomplete, unsigned int maxNeigh,  int* neighbourhoods, float wL_, float * wH_)
{
	printf("openCL_LaplaceContraction");
	printf("\n");

	#ifdef _LOG
		Timerlog timerlog = Timerlog("openCL_LaplaceContraction");
	#endif

	int count = numOfVertices * 3;
	float * wL = new float[numOfVertices];
	float * wH = new float[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		wL[i] = wL_;
		wH[i] = wH_[i];
	}

	/*

	- kazdy kernel bude ratat novu poziciu jedneho vrchola, 3xN threadov pre kazdu suradnicu
		- nahrame maticu L na GPU
		- vytvorime matice L1 ... LN pre okolie kazdeho bodu lokalne
		- z kazdej Li spravime matice Qi a Ri
		- spustime paralelny vypocet
		- zapisemu novu poziciu vrchola
	*/


	#ifdef _LOG
		timerlog.addStart("Kernel prepairing and building");
	#endif

	char *KernelSource = (char*)loadKernelSource((char*)(projectDir + std::string("OpenCLKernels\\kernel-QRlocal.cl")).c_str());

	// replace value of MAX_SIZE_NEIGHBOURHOOD
	std::string stringToReplace = std::string(KernelSource);
	int idx = stringToReplace.find("const int MAX_SIZE_NEIGHBOURHOOD = $;");

	if (idx > 0){
		char strBuffer[255] = {0};
		sprintf_s(strBuffer, sizeof(strBuffer), "const int MAX_SIZE_NEIGHBOURHOOD = %i", maxNeigh);
		std::string textNeigh = std::string(strBuffer);

		for (int i=0; i < textNeigh.size(); i++)
			KernelSource[idx + i] = strBuffer[i];

		printf(strBuffer);
		printf("\n");
	}

	//const char *KernelSource = (const char*)tempKernelSource;

	int err;                            // error code returned from api calls

	float * results = new float[count];           // results returned from device
	unsigned int correct;               // number of correct results returned

	size_t global;                      // global domain size for our calculation
	size_t local;                       // local domain size for our calculation
	cl_device_id device_id;             // compute device id 
	cl_context context;                 // compute context
	cl_command_queue commands;          // compute command queue
	cl_program program;                 // compute program
	cl_kernel kernel;                   // compute kernel

	// Connect to a compute device

	//CL_DEVICE_TYPE_GPU
	//CL_DEVICE_TYPE_CPU
	//CL_DEVICE_TYPE_DEFAULT

	//printCLInfo();

	cl_int clStatus;
	cl_uint numPlatforms;
	cl_platform_id platform_id;
	clStatus = clGetPlatformIDs(1, &platform_id, &numPlatforms);

	err = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, 1, &device_id, NULL);
	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to create a device group!\n");
	}

	// Create a compute context 
	context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);

	if (!context)
	{
		printf("Error: Failed to create a compute context!\n");
	}

	// Create a command commands

	commands = clCreateCommandQueue(context, device_id, 0, &err);

	if (!commands)
	{
		printf("Error: Failed to create a command commands!\n");
	}

	// Create the compute program from the source buffer

	program = clCreateProgramWithSource(context, 1, (const char **) & KernelSource, NULL, &err);

	if (!program)
	{
		printf("Error: Failed to create compute program!\n");
	}

	// Build the program executable

	err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
	if (err != CL_SUCCESS)
	{
		size_t len;
		char buffer[2480];

		printf("Error: Failed to build program executable!\n");
		clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, NULL);
		printf("%s\n", buffer);
	}

	// Create the compute kernel in the program we wish to run

	kernel = clCreateKernel(program, "laplace", &err);

	if (!kernel || err != CL_SUCCESS)

	{
		printf("Error: Failed to create compute kernel!\n");
	}

	// Create the input and output arrays in device memory for our calculation

	cl_mem input_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * count, NULL, NULL);
	cl_mem output_GPU = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(float) * count, NULL, NULL);
	cl_mem L_complete_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices * numOfVertices, NULL, NULL);
	//cl_mem L_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * maxNeigh * maxNeigh, NULL, NULL);
	//cl_mem B_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * (2 * maxNeigh), NULL, NULL);
	cl_mem neighbourhoods_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(int) * numOfVertices * (maxNeigh + 1), NULL, NULL);
	cl_mem WL_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices, NULL, NULL);
	cl_mem WH_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices, NULL, NULL);
	//cl_mem QR_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * maxNeigh * (2 * maxNeigh), NULL, NULL);
	//cl_mem Rdiag_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * (2 * maxNeigh), NULL, NULL);
	//cl_mem X_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * maxNeigh, NULL, NULL);

	if (!input_GPU || !output_GPU)

	{
		printf("Error: Failed to allocate device memory!\n");
	}    

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("prepairing input arrays");
	#endif

	// Write our data set into the input array in device memory 

	err = clEnqueueWriteBuffer(commands, L_complete_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices * numOfVertices, Lcomplete, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, neighbourhoods_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * (maxNeigh + 1), neighbourhoods, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, WL_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices, wL, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, WH_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices, wH, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, input_GPU, CL_FALSE, 0, sizeof(float) * count, input, 0, NULL, NULL);

	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to write to source array!\n");
	}

	// Set the arguments to our compute kernel

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("settings kernel arguments");
	#endif

	err = 0;
	// global variables
	err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input_GPU);
	err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &output_GPU);
	err |= clSetKernelArg(kernel, 2, sizeof(unsigned int), &numOfVertices);
	err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &L_complete_GPU);
	//err |= clSetKernelArg(kernel, 4, sizeof(unsigned int), &maxNeigh);
	err |= clSetKernelArg(kernel, 4, sizeof(cl_mem), &neighbourhoods_GPU);
	err |= clSetKernelArg(kernel, 5, sizeof(cl_mem), &WL_GPU);
	err |= clSetKernelArg(kernel, 6, sizeof(cl_mem), &WH_GPU);
	// local variables
	/*err |= clSetKernelArg(kernel, 8, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 9, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 10, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 11, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 12, sizeof(cl_mem), NULL);*/

	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to set kernel arguments! %d\n", err);
	}



	// Get the maximum work group size for executing the kernel on the device

	//

	err = clGetKernelWorkGroupInfo(kernel, device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), &local, NULL);

	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to retrieve kernel work group info! %d\n", err);

	}


	// Execute the kernel over the entire range of our 1d input data set

	// using the maximum number of work group items for this device

	global = (count > 1) ? FloorPow2(count) : count;

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("starting kernel");
	#endif

	//err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, NULL, 0, NULL, NULL);
	err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL);

	if (err)
	{
		printf("Error: Failed to execute kernel!\n");

	}

	// Wait for the command commands to get serviced before reading back results

	clFinish(commands);


	// Read back the results from the device to verify the output

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("reading back results");
	#endif

	float * outputLocal = new float[3 * neighbourhoods[0]];

	err = clEnqueueReadBuffer( commands, output_GPU, CL_TRUE, 0, sizeof(float) * count, results, 0, NULL, NULL );  

	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to read output array! %d\n", err);
	}

	for (int i=0; i < numOfVertices; i++){
		output[i] = CVector3(results[i], results[numOfVertices + i], results[2*numOfVertices + i]);
	}

	// Shutdown and cleanup

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("releasing memory");
	#endif

	clReleaseMemObject(input_GPU);
	clReleaseMemObject(output_GPU);
	clReleaseMemObject(L_complete_GPU);
	//clReleaseMemObject(L_GPU);
	//clReleaseMemObject(B_GPU);
	clReleaseMemObject(WL_GPU);
	clReleaseMemObject(WH_GPU);
	//clReleaseMemObject(X_GPU);
	//clReleaseMemObject(Rdiag_GPU);
	clReleaseMemObject(neighbourhoods_GPU);
	//clReleaseMemObject(QR_GPU);

	clReleaseProgram(program);
	clReleaseKernel(kernel);
	clReleaseCommandQueue(commands);
	clReleaseContext(context);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.logExecutionTimes();
	#endif
}

void OpenCLManager::initializeInteropContext(OpenCLContext * oclc){
	cl_platform_id cpPlatform; // OpenCL platform
	cl_int err;     // error code returned from api calls
	int devType = CL_DEVICE_TYPE_GPU;

	err = clGetPlatformIDs(1, &cpPlatform, NULL);

	// Get a device of the appropriate type
	err = clGetDeviceIDs(cpPlatform, devType, 1, &(oclc->device_id), NULL);
	if (err != CL_SUCCESS) {
	}

	// Get string containing supported device extensions
	size_t ext_size = 1024;
	char* ext_string = (char*)malloc(ext_size);
	err = clGetDeviceInfo(oclc->device_id, CL_DEVICE_EXTENSIONS, ext_size, ext_string, &ext_size);

	// Create CL context properties, add WGL context & handle to DC

	cl_context_properties properties[] = {
		// UNCOMMENT UNDER WINDOWS
		//CL_GL_CONTEXT_KHR, (cl_context_properties)wglGetCurrentContext(), // WGL Context
		//CL_WGL_HDC_KHR, (cl_context_properties)wglGetCurrentDC(), // WGL HDC
		CL_CONTEXT_PLATFORM, (cl_context_properties)cpPlatform, // OpenCL platform
		0
	};

	// Find CL capable devices in the current GL context
	cl_device_id devices[32]; size_t sizedev = 0;

	// zakomanetovane lebo Miso nevedel toto resolvnut
	//clGetGLContextInfoKHR_fn clGetGLContextInfo = (clGetGLContextInfoKHR_fn)clGetExtensionFunctionAddressForPlatform(cpPlatform, "clGetGLContextInfoKHR");
	//clGetGLContextInfo(properties, CL_DEVICES_FOR_GL_CONTEXT_KHR, 32 * sizeof(cl_device_id), devices, &sizedev);

	// Create a context using the supported devices
	cl_uint countdev = (cl_uint)(sizedev / sizeof(cl_device_id));
	oclc->context = clCreateContext(properties, countdev, devices, NULL, 0, 0);
}

void OpenCLManager::openCL_LaplaceContractionInterop(int * ite, CVector3 * output, float* input, unsigned int numOfVertices, float * Ecomplete, unsigned int maxNeigh,  int* neighbourhoods,  unsigned int maxNeigh2,  int* neighbourhoods2, float wL_, float * wH_, float sL, int numOfIte, int vboID, OpenCLContext oclc){
	printf("openCL_LaplaceContractionIterative");
	printf("\n");

	#ifdef _LOG
		Timerlog timerlog = Timerlog("openCL_LaplaceContractionIterative");
	#endif

	int count = numOfVertices * 3;
	float * wL = new float[numOfVertices];
	float * wH = new float[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		wL[i] = wL_;
		wH[i] = wH_[i];
	}

	/*

	- kazdy kernel bude ratat novu poziciu jedneho vrchola, 3xN threadov pre kazdu suradnicu
		- nahrame maticu L na GPU
		- vytvorime matice L1 ... LN pre okolie kazdeho bodu lokalne
		- z kazdej Li spravime matice Qi a Ri
		- spustime paralelny vypocet
		- zapisemu novu poziciu vrchola
	*/


	#ifdef _LOG
		timerlog.addStart("Kernel - loading");
	#endif

	char *KernelSource = (char*)loadKernelSource((char*)(projectDir + std::string("OpenCLKernels\\kernel-QRlocalInterop.cl")).c_str());

	// replace value of MAX_SIZE_NEIGHBOURHOOD
	std::string stringToReplace = std::string(KernelSource);
	int idx = stringToReplace.find("const int MAX_SIZE_NEIGHBOURHOOD = $;");

	if (idx > 0){
		char strBuffer[255] = {0};
		sprintf_s(strBuffer, sizeof(strBuffer), "const int MAX_SIZE_NEIGHBOURHOOD = %i;", maxNeigh);
		std::string textNeigh = std::string(strBuffer);

		for (int i=0; i < textNeigh.size(); i++)
			KernelSource[idx + i] = strBuffer[i];

		printf(strBuffer);
		printf("\n");
	}


	// replace value of MAX_SIZE_NEIGHBOURHOOD2
	std::string stringToReplace2 = std::string(KernelSource);
	int idx2 = stringToReplace.find("const int MAX_SIZE_NEIGHBOURHOOD2 = $;");

	if (idx2 > 0){
		char strBuffer2[255] = {0};
		sprintf_s(strBuffer2, sizeof(strBuffer2), "const int MAX_SIZE_NEIGHBOURHOOD2 = %i;", maxNeigh2);
		std::string textNeigh2 = std::string(strBuffer2);

		for (int i=0; i < textNeigh2.size(); i++)
			KernelSource[idx2 + i] = strBuffer2[i];

		printf(strBuffer2);
		printf("\n");
	}

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - context setting");
	#endif

	cl_command_queue commands; // compute command queue
	cl_program program;        // compute program
	cl_kernel kernel;          // compute kernel
	int devType = CL_DEVICE_TYPE_GPU;

	cl_int err;     // error code returned from api calls

	float * results = new float[count];           // results returned from device
	unsigned int correct;               // number of correct results returned

	size_t global;                      // global domain size for our calculation
	size_t local;                       // local domain size for our calculation

	// Create a compute context 
	//context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &err);

    //printCLInfo();


	if (!oclc.context)
	{
		printf("Error: Failed to create a compute context!\n");
	}

	// Create a command commands

	commands = clCreateCommandQueue(oclc.context, oclc.device_id, 0, &err);

	if (!commands)
	{
		printf("Error: Failed to create a command commands!\n");
	}

	// Create the compute program from the source buffer

	program = clCreateProgramWithSource(oclc.context, 1, (const char **) & KernelSource, NULL, &err);

	if (!program)
	{
		printf("Error: Failed to create compute program!\n");
	}

	// Build the program executable

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - bulding program");
	#endif

	err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
	if (err != CL_SUCCESS)
	{
		size_t len;
		char buffer[2480];

		printf("Error: Failed to build program executable!\n");
		clGetProgramBuildInfo(program, oclc.device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
		printf("%s\n", buffer);
	}

	// Create the compute kernel in the program we wish to run

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - creating");
	#endif

	kernel = clCreateKernel(program, "laplace", &err);

	if (!kernel || err != CL_SUCCESS)

	{
		printf("Error: Failed to create compute kernel!\n");
	}

	// Create the input and output arrays in device memory for our calculation

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - setting memmory");
	#endif

	//cl_mem input_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * count, NULL, NULL);
	//cl_mem output_GPU = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(float) * count, NULL, NULL);
	cl_mem E_complete_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices * numOfVertices, NULL, NULL);
	cl_mem neighbourhoodsMinor_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(int) * numOfVertices * (maxNeigh + 1), NULL, NULL);
	cl_mem neighbourhoodsMajor_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(int) * numOfVertices * (maxNeigh2 + 1), NULL, NULL);
	cl_mem WL_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices, NULL, NULL);
	cl_mem WH_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices, NULL, NULL);
	// opencl VBO
	cl_mem vbo_cl = clCreateFromGLBuffer(oclc.context, CL_MEM_READ_WRITE, vboID, NULL);


	/*if (!input_GPU)

	{
		printf("Error: Failed to allocate device memory!\n");
	}    */

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("prepairing input arrays");
	#endif

	// Write our data set into the input array in device memory 

	err = clEnqueueWriteBuffer(commands, E_complete_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices * numOfVertices, Ecomplete, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, neighbourhoodsMinor_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * (maxNeigh + 1), neighbourhoods, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, neighbourhoodsMajor_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * (maxNeigh2 + 1), neighbourhoods2, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, WL_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices, wL, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, WH_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices, wH, 0, NULL, NULL);
//	err |= clEnqueueWriteBuffer(commands, input_GPU, CL_FALSE, 0, sizeof(float) * count, input, 0, NULL, NULL);

	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to write to source array!\n");
	}

	// Set the arguments to our compute kernel

#ifdef _LOG
	timerlog.addEnd();
	timerlog.addStart("settings kernel arguments");
#endif

	err = 0;
	// global variables
	//err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input_GPU);
	err |= clSetKernelArg(kernel, 0, sizeof(cl_mem), &vbo_cl);
	err |= clSetKernelArg(kernel, 1, sizeof(unsigned int), &numOfVertices);
	err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &E_complete_GPU);
	err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &neighbourhoodsMinor_GPU);
	err |= clSetKernelArg(kernel, 4, sizeof(cl_mem), &neighbourhoodsMajor_GPU);
	err |= clSetKernelArg(kernel, 5, sizeof(cl_mem), &WL_GPU);
	err |= clSetKernelArg(kernel, 6, sizeof(cl_mem), &WH_GPU);

	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to set kernel arguments! %d\n", err);
	}

	// Get the maximum work group size for executing the kernel on the device

	//

	err = clGetKernelWorkGroupInfo(kernel, oclc.device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), &local, NULL);
	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to retrieve kernel work group info! %d\n", err);

	}
	// Execute the kernel over the entire range of our 1d input data set

	// using the maximum number of work group items for this device

	global = (count > 1) ? FloorPow2(count) : count;

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	while (*ite <= numOfIte){

#ifdef _LOG
		timerlog.addStart("iteration");
#endif
		printf("Iteration : %i\n", *ite);

		//glFinish();
		//clEnqueueAcquireGLObjects(commands, 1, &vbo_cl, 0,0,0);
		err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL);
		//clEnqueueReleaseGLObjects(commands, 1, &vbo_cl, 0,0,0);
		clFinish(commands);

		if (err)
		{
			printf("Error: Failed to execute kernel!\n");

		}

		// Wait for the command commands to get serviced before reading back results

		// Read back the results from the device to verify the output
		#ifdef _LOG
				timerlog.addEnd();
		#endif

		if (oclc.transferInteroperabilityMesh){
#ifdef _LOG
			timerlog.addStart("reading back results");
#endif

			err = clEnqueueReadBuffer( commands, vbo_cl, CL_TRUE, 0, sizeof(float) * count, results, 0, NULL, NULL );  

			if (err != CL_SUCCESS)
			{
			printf("Error: Failed to read output array! %d\n", err);
			}

			for (int i=0; i < numOfVertices; i++){
			output[i] = CVector3(results[i * 3], results[i * 3 + 1], results[i * 3 + 2]);
			}

				// Shutdown and cleanup
		#ifdef _LOG
			timerlog.addEnd();
#endif
		}

		(*ite)++;
	}

	#ifdef _LOG
		timerlog.addStart("releasing memory");
	#endif

	clReleaseMemObject(vbo_cl);
	clReleaseMemObject(E_complete_GPU);
	clReleaseMemObject(WL_GPU);
	clReleaseMemObject(WH_GPU);
	clReleaseMemObject(neighbourhoodsMinor_GPU);
	clReleaseMemObject(neighbourhoodsMajor_GPU);
	clReleaseProgram(program);
	clReleaseKernel(kernel);
	clReleaseCommandQueue(commands);
	clReleaseContext(oclc.context);

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	#ifdef _LOG
		timerlog.logExecutionTimes();
	#endif
}

void OpenCLManager::openCL_LaplaceContraction2ring(CVector3 * output, float* input, unsigned int numOfVertices, float * Lcomplete, unsigned int maxNeigh,  int* neighbourhoods, unsigned int maxNeigh2,  int* neighbourhoods2, float wL_, float * wH_)
{
	printf("openCL_LaplaceContraction2ring");
	printf("\n");

#ifdef _LOG
	Timerlog timerlog = Timerlog("openCL_LaplaceContraction");
#endif

	int count = numOfVertices * 3;
	float * wL = new float[numOfVertices];
	float * wH = new float[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		wL[i] = wL_;
		wH[i] = wH_[i];
	}

	/*

	- kazdy kernel bude ratat novu poziciu jedneho vrchola, 3xN threadov pre kazdu suradnicu
		- nahrame maticu L na GPU
		- vytvorime matice L1 ... LN pre okolie kazdeho bodu lokalne
		- z kazdej Li spravime matice Qi a Ri
		- spustime paralelny vypocet
		- zapisemu novu poziciu vrchola
	*/


	#ifdef _LOG
		timerlog.addStart("Kernel prepairing and building");
	#endif

	char *KernelSource = (char*)loadKernelSource((char*)(solutionDir + std::string("OpenCLKernels\\kernel-QRlocal2ring.cl")).c_str());

	// replace value of MAX_SIZE_NEIGHBOURHOOD
	std::string stringToReplace = std::string(KernelSource);
	int idx = stringToReplace.find("const int MAX_SIZE_NEIGHBOURHOOD = $;");

	if (idx > 0){
		char strBuffer[255] = {0};
		sprintf_s(strBuffer, sizeof(strBuffer), "const int MAX_SIZE_NEIGHBOURHOOD = %i;", maxNeigh);
		std::string textNeigh = std::string(strBuffer);

		for (int i=0; i < textNeigh.size(); i++)
			KernelSource[idx + i] = strBuffer[i];

		printf(strBuffer);
		printf("\n");
	}

	// replace value of MAX_SIZE_NEIGHBOURHOOD2
	std::string stringToReplace2 = std::string(KernelSource);
	int idx2 = stringToReplace.find("const int MAX_SIZE_NEIGHBOURHOOD2 = $;");

	if (idx2 > 0){
		char strBuffer2[255] = {0};
		sprintf_s(strBuffer2, sizeof(strBuffer2), "const int MAX_SIZE_NEIGHBOURHOOD2 = %i;", maxNeigh2);
		std::string textNeigh2 = std::string(strBuffer2);

		for (int i=0; i < textNeigh2.size(); i++)
			KernelSource[idx2 + i] = strBuffer2[i];

		printf(strBuffer2);
		printf("\n");
	}

	//const char *KernelSource = (const char*)tempKernelSource;

	int err;                            // error code returned from api calls

	float * results = new float[count];           // results returned from device
	unsigned int correct;               // number of correct results returned

	size_t global;                      // global domain size for our calculation
	size_t local;                       // local domain size for our calculation
	cl_device_id device_id;             // compute device id 
	cl_context context;                 // compute context
	cl_command_queue commands;          // compute command queue
	cl_program program;                 // compute program
	cl_kernel kernel;                   // compute kernel

	// Connect to a compute device

	//CL_DEVICE_TYPE_GPU
	//CL_DEVICE_TYPE_CPU
	//CL_DEVICE_TYPE_DEFAULT

	//printCLInfo();

	cl_int clStatus;
	cl_uint numPlatforms;
	cl_platform_id platform_id;
	clStatus = clGetPlatformIDs(1, &platform_id, &numPlatforms);

	err = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, 1, &device_id, NULL);
	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to create a device group!\n");
	}

	// Create a compute context 
	context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);

	if (!context)
	{
		printf("Error: Failed to create a compute context!\n");
	}

	// Create a command commands

	commands = clCreateCommandQueue(context, device_id, 0, &err);

	if (!commands)
	{
		printf("Error: Failed to create a command commands!\n");
	}

	// Create the compute program from the source buffer

	program = clCreateProgramWithSource(context, 1, (const char **) & KernelSource, NULL, &err);

	if (!program)
	{
		printf("Error: Failed to create compute program!\n");
	}

	// Build the program executable

	err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
	if (err != CL_SUCCESS)
	{
		size_t len;
		char buffer[2480];

		printf("Error: Failed to build program executable!\n");
		clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
		printf("%s\n", buffer);
	}

	// Create the compute kernel in the program we wish to run

	kernel = clCreateKernel(program, "laplace", &err);

	if (!kernel || err != CL_SUCCESS)

	{
		printf("Error: Failed to create compute kernel!\n");
	}

	// Create the input and output arrays in device memory for our calculation

	cl_mem input_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * count, NULL, NULL);
	cl_mem output_GPU = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(float) * count, NULL, NULL);
	cl_mem L_complete_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices * numOfVertices, NULL, NULL);
	//cl_mem L_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * maxNeigh * maxNeigh, NULL, NULL);
	//cl_mem B_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * (2 * maxNeigh), NULL, NULL);
	cl_mem neighbourhoodsMinor_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(int) * numOfVertices * (maxNeigh + 1), NULL, NULL);
	cl_mem neighbourhoodsMajor_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(int) * numOfVertices * (maxNeigh2 + 1), NULL, NULL);
	cl_mem WL_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices, NULL, NULL);
	cl_mem WH_GPU = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices, NULL, NULL);
	//cl_mem QR_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * maxNeigh * (2 * maxNeigh), NULL, NULL);
	//cl_mem Rdiag_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * (2 * maxNeigh), NULL, NULL);
	//cl_mem X_GPU = clCreateBuffer(context,  CL_MEM_READ_WRITE,  sizeof(float) * maxNeigh, NULL, NULL);

	if (!input_GPU || !output_GPU)

	{
		printf("Error: Failed to allocate device memory!\n");
	}    

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("prepairing input arrays");
	#endif

	// Write our data set into the input array in device memory 

	err = clEnqueueWriteBuffer(commands, L_complete_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices * numOfVertices, Lcomplete, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, neighbourhoodsMinor_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * (maxNeigh + 1), neighbourhoods, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, neighbourhoodsMajor_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * (maxNeigh2 + 1), neighbourhoods2, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, WL_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices, wL, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, WH_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices, wH, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, input_GPU, CL_FALSE, 0, sizeof(float) * count, input, 0, NULL, NULL);

	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to write to source array!\n");
	}

	// Set the arguments to our compute kernel

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("settings kernel arguments");
	#endif

	err = 0;
	// global variables
	err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input_GPU);
	err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &output_GPU);
	err |= clSetKernelArg(kernel, 2, sizeof(unsigned int), &numOfVertices);
	err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &L_complete_GPU);
	err |= clSetKernelArg(kernel, 4, sizeof(unsigned int), &maxNeigh);
	err |= clSetKernelArg(kernel, 5, sizeof(cl_mem), &neighbourhoodsMinor_GPU);
	err |= clSetKernelArg(kernel, 6, sizeof(unsigned int), &maxNeigh2);
	err |= clSetKernelArg(kernel, 7, sizeof(cl_mem), &neighbourhoodsMajor_GPU);
	err |= clSetKernelArg(kernel, 8, sizeof(cl_mem), &WL_GPU);
	err |= clSetKernelArg(kernel, 9, sizeof(cl_mem), &WH_GPU);
	// local variables
	/*err |= clSetKernelArg(kernel, 8, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 9, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 10, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 11, sizeof(cl_mem), NULL);
	err |= clSetKernelArg(kernel, 12, sizeof(cl_mem), NULL);*/

	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to set kernel arguments! %d\n", err);
	}



	// Get the maximum work group size for executing the kernel on the device

	//

	err = clGetKernelWorkGroupInfo(kernel, device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), &local, NULL);

	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to retrieve kernel work group info! %d\n", err);

	}


	// Execute the kernel over the entire range of our 1d input data set

	// using the maximum number of work group items for this device

	global = (count > 1) ? FloorPow2(count) : count;

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("starting kernel");
	#endif

	//err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, NULL, 0, NULL, NULL);
	err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL);

	if (err)
	{
		printf("Error: Failed to execute kernel!\n");

	}

	// Wait for the command commands to get serviced before reading back results

	clFinish(commands);


	// Read back the results from the device to verify the output

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("reading back results");
	#endif

	float * outputLocal = new float[3 * neighbourhoods[0]];

	err = clEnqueueReadBuffer( commands, output_GPU, CL_TRUE, 0, sizeof(float) * count, results, 0, NULL, NULL );  

	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to read output array! %d\n", err);
	}

	for (int i=0; i < numOfVertices; i++){
		output[i] = CVector3(results[i], results[numOfVertices + i], results[2*numOfVertices + i]);
	}

	// Shutdown and cleanup

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("releasing memory");
	#endif

	clReleaseMemObject(input_GPU);
	clReleaseMemObject(output_GPU);
	clReleaseMemObject(L_complete_GPU);
	//clReleaseMemObject(L_GPU);
	//clReleaseMemObject(B_GPU);
	clReleaseMemObject(WL_GPU);
	clReleaseMemObject(WH_GPU);
	//clReleaseMemObject(X_GPU);
	//clReleaseMemObject(Rdiag_GPU);
	clReleaseMemObject(neighbourhoodsMinor_GPU);
	clReleaseMemObject(neighbourhoodsMajor_GPU);
	//clReleaseMemObject(QR_GPU);

	clReleaseProgram(program);
	clReleaseKernel(kernel);
	clReleaseCommandQueue(commands);
	clReleaseContext(context);

	#ifdef _LOG

		timerlog.addEnd();

		timerlog.logExecutionTimes();
	#endif
}

void OpenCLManager::openCL_LaplaceContractionJacobi(CVector3 * output, float* input, unsigned int numOfVertices, float * ASquared, float * B, int vboID, OpenCLContext oclc){
	printf("openCL_LaplaceContractionJacobi");
	printf("\n");
	#ifdef _LOG
		Timerlog timerlog = Timerlog("openCL_LaplaceContractionJacobi");
	#endif

	/*for (int i=0; i < 30; i++)
		printf("%f ", input[i]);
	printf("\n ");
	for (int i=0; i < 30; i++)
		printf("%f ", ASquared[i]);
	printf("\n ");
	for (int i=0; i < 30; i++)
		printf("%f ", B[i]);
	printf("\n ");*/

	/*float ASquared[9] = {10.0, 2.0, -1.0,
		1.0, 8.0, 3.0, 
		-2.0, -1.0, 10.0};
	float B[9] = {7.0, 7.0, 7.0,
				-4.0, -4.0, -4.0,
				9.0, 9.0, 9.0};
	float xxx[9] = {0.0, 0.0, 0.0,
					0.0, 0.0, 0.0,
					0.0, 0.0, 0.0};*/

	//int numOfVertices = 3;

	int count = numOfVertices * 3;

	float * wL = new float[numOfVertices];
	float * wH = new float[numOfVertices];
	/*for (int i=0; i < numOfVertices; i++){
		wL[i] = wL_;
		wH[i] = wH_[i];
	}*/


	#ifdef _LOG
		timerlog.addStart("Kernel - loading");
	#endif

	char *KernelSource = (char*)loadKernelSource((char*)(solutionDir + std::string("OpenCLKernels\\kernel-Jacobi.cl")).c_str());

	
	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - context setting");
	#endif

	cl_command_queue commands; // compute command queue
	cl_program program;        // compute program
	cl_kernel kernel;          // compute kernel
	int devType = CL_DEVICE_TYPE_GPU;

	cl_int err;     // error code returned from api calls

	float * results = new float[count];           // results returned from device
	
	float * xxx = new float[count];
	for (int i=0; i < count; i++){
		xxx[i] = input[i];
		results[i] = input[i];
	}


	unsigned int correct;               // number of correct results returned

	size_t global;                      // global domain size for our calculation
	size_t local;                       // local domain size for our calculation


    //printCLInfo();

	
	// Create a compute context 
	oclc.context = clCreateContext(0, 1, &oclc.device_id, NULL, NULL, &err);


	if (!oclc.context)
	{
		printf("Error: Failed to create a compute context!\n");
	}

	// Create a command commands

	commands = clCreateCommandQueue(oclc.context, oclc.device_id, 0, &err);

	if (!commands)
	{
		printf("Error: Failed to create a command commands!\n");
	}

	// Create the compute program from the source buffer

	program = clCreateProgramWithSource(oclc.context, 1, (const char **) & KernelSource, NULL, &err);

	if (!program)
	{
		printf("Error: Failed to create compute program!\n");
	}

	// Build the program executable

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - bulding program");
	#endif

	err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
	if (err != CL_SUCCESS)
	{
		size_t len;
		char buffer[2480];

		printf("Error: Failed to build program executable!\n");
		clGetProgramBuildInfo(program, oclc.device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
		printf("%s\n", buffer);
	}

	// Create the compute kernel in the program we wish to run

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - creating");
	#endif

	kernel = clCreateKernel(program, "laplace", &err);

	if (!kernel || err != CL_SUCCESS)

	{
		printf("Error: Failed to create compute kernel!\n");
	}

	// Create the input and output arrays in device memory for our calculation

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("Kernel - setting memmory");
	#endif

	cl_mem Asquared_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices * numOfVertices, NULL, NULL);
	cl_mem B_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices * 3, NULL, NULL);
	cl_mem X_GPU = clCreateBuffer(oclc.context,  CL_MEM_READ_ONLY,  sizeof(float) * numOfVertices * 3, NULL, NULL);

	// opencl VBO
	//cl_mem vbo_cl = clCreateFromGLBuffer(oclc.context, CL_MEM_READ_WRITE, vboID, NULL);


	/*if (!input_GPU)

	{
		printf("Error: Failed to allocate device memory!\n");
	}    */

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("prepairing input arrays");
	#endif

	// Write our data set into the input array in device memory 

	err = clEnqueueWriteBuffer(commands, Asquared_GPU, CL_FALSE, 0, sizeof(float) * numOfVertices * numOfVertices, ASquared, 0, NULL, NULL);
	err |= clEnqueueWriteBuffer(commands, B_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * 3, B, 0, NULL, NULL);
	//err |= clEnqueueWriteBuffer(commands, X_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices, xxx, 0, NULL, NULL);

	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to write to source array!\n");
	}

	// Set the arguments to our compute kernel

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("settings kernel arguments");
	#endif

	err = 0;
	// global variables
	err |= clSetKernelArg(kernel, 0, sizeof(cl_mem), &X_GPU);
	err |= clSetKernelArg(kernel, 1, sizeof(unsigned int), &numOfVertices);
	err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &Asquared_GPU);
	err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &B_GPU);

	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to set kernel arguments! %d\n", err);
	}

	// Get the maximum work group size for executing the kernel on the device

	//

	err = clGetKernelWorkGroupInfo(kernel, oclc.device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), &local, NULL);
	if (err != CL_SUCCESS)

	{
		printf("Error: Failed to retrieve kernel work group info! %d\n", err);

	}
	// Execute the kernel over the entire range of our 1d input data set

	// using the maximum number of work group items for this device

	global = (count > 1) ? FloorPow2(count) : count;
	if (global < local)
		local = global;

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	float delta = FLT_MAX;
	int ittt = 0;

	for (int i=0; i < 9; i++){
		results[i] = xxx[i];
	}

	while (abs(delta) > 0.0001){
	//while (ittt < 4){
	#ifdef _LOG
		timerlog.addStart("iteration");
	#endif
		printf("Iteracia : %i\n", ittt);
		ittt++;

			printf("writujeme X: %f %f %f \n", results[0], results[3], results[6]);
			printf("writujeme Y: %f %f %f \n", results[1], results[4], results[7]);
			printf("writujeme Z: %f %f %f \n", results[2], results[5], results[8]);

		//clEnqueueAcquireGLObjects(commands, 1, &vbo_cl, 0,0,0);
		err = clEnqueueWriteBuffer(commands, X_GPU, CL_FALSE, 0, sizeof(int) * numOfVertices * 3, results, 0, NULL, NULL);
		err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL);
		//clEnqueueReleaseGLObjects(commands, 1, &vbo_cl, 0,0,0);
		clFinish(commands);

		if (err)
		{
			printf("Error: Failed to execute kernel!\n");

		}

		// Wait for the command commands to get serviced before reading back results

		// Read back the results from the device to verify the output
#ifdef _LOG
		timerlog.addEnd();
#endif

			delta = 0.0;

			err = clEnqueueReadBuffer( commands, X_GPU, CL_TRUE, 0, sizeof(float) * count, results, 0, NULL, NULL );  

			if (err != CL_SUCCESS)
			{
			printf("Error: Failed to read output array! %d\n", err);
			}

			for (int i=0; i < count; i++){
				delta += results[i] - xxx[i];
			}

			for (int i=0; i < count; i++){
				xxx[i] = results[i];
			}

			printf("Results X: %f %f %f \n", results[0], results[3], results[6]);
			printf("Results Y: %f %f %f \n", results[1], results[4], results[7]);
			printf("Results Z: %f %f %f \n", results[2], results[5], results[8]);

			delta /= numOfVertices * 3;

			printf("Delta : %f\n", delta);
			
			// Shutdown and cleanup
#ifdef _LOG
			timerlog.addEnd();
#endif

	}

	for (int i=0; i < numOfVertices; i++){
		output[i] = CVector3(results[i * 3], results[i * 3 + 1], results[i * 3 + 2]);
	}

	#ifdef _LOG
		timerlog.addStart("releasing memory");
	#endif

	clReleaseMemObject(X_GPU);
	clReleaseMemObject(Asquared_GPU);
	clReleaseMemObject(B_GPU);
	clReleaseProgram(program);
	clReleaseKernel(kernel);
	clReleaseCommandQueue(commands);
	clReleaseContext(oclc.context);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.logExecutionTimes();
	#endif
}



/*Display OpenCL system info */
void OpenCLManager::printCLInfo()
{
	size_t p_size;
	size_t arr_tsize[3];
	size_t ret_size;
	char param[100];
	cl_uint entries;
	cl_ulong long_entries;
	cl_bool bool_entries;
	cl_uint num_devices;
	cl_device_local_mem_type mem_type;
	cl_device_type dev_type;
	cl_device_fp_config fp_conf;
	cl_device_exec_capabilities exec_cap;
	cl_device_id device_id;        

	cl_int clStatus;
	cl_uint numPlatforms;
	cl_platform_id platform_id;
	clStatus = clGetPlatformIDs(1, &platform_id, &numPlatforms);

	int err = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, 1, &device_id, NULL);
	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to create a device group!\n");
	}


		clGetDeviceInfo(device_id ,CL_DEVICE_TYPE,sizeof(dev_type),&dev_type,&ret_size);
		printf("\tDevice Type:\t\t");
		if(dev_type & CL_DEVICE_TYPE_GPU)
			printf("CL_DEVICE_TYPE_GPU ");
		if(dev_type & CL_DEVICE_TYPE_CPU)
			printf("CL_DEVICE_TYPE_CPU ");
		if(dev_type & CL_DEVICE_TYPE_ACCELERATOR)
			printf("CL_DEVICE_TYPE_ACCELERATOR ");
		if(dev_type & CL_DEVICE_TYPE_DEFAULT)
			printf("CL_DEVICE_TYPE_DEFAULT ");
		printf("\n");


		clGetDeviceInfo(device_id,CL_DEVICE_NAME,sizeof(param),param,&ret_size);
		printf("\tName: \t\t\t%s\n",param);

		clGetDeviceInfo(device_id,CL_DEVICE_VENDOR,sizeof(param),param,&ret_size);
		printf("\tVendor: \t\t%s\n",param);

		clGetDeviceInfo(device_id,CL_DEVICE_VENDOR_ID,sizeof(cl_uint),&entries,&ret_size);
		printf("\tVendor ID:\t\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_DEVICE_VERSION,sizeof(param),param,&ret_size);
		printf("\tVersion:\t\t%s\n",param);

		clGetDeviceInfo(device_id,CL_DEVICE_PROFILE,sizeof(param),param,&ret_size);
		printf("\tProfile:\t\t%s\n",param);

		clGetDeviceInfo(device_id,CL_DRIVER_VERSION,sizeof(param),param,&ret_size);
		printf("\tDriver: \t\t%s\n",param);

		clGetDeviceInfo(device_id,CL_DEVICE_EXTENSIONS,sizeof(param),param,&ret_size);
		printf("\tExtensions:\t\t%s\n",param);

		clGetDeviceInfo(device_id,CL_DEVICE_MAX_WORK_ITEM_SIZES,3*sizeof(size_t),arr_tsize,&ret_size);
		printf("\tMax Work-Item Sizes:\t(%d,%d,%d)\n",arr_tsize[0],arr_tsize[1],arr_tsize[2]);

		clGetDeviceInfo(device_id,CL_DEVICE_MAX_WORK_GROUP_SIZE,sizeof(size_t),&p_size,&ret_size);
		printf("\tMax Work Group Size:\t%d\n",p_size);

		clGetDeviceInfo(device_id,CL_DEVICE_MAX_COMPUTE_UNITS,sizeof(cl_uint),&entries,&ret_size);
		printf("\tMax Compute Units:\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_KERNEL_WORK_GROUP_SIZE,sizeof(cl_uint),&entries,&ret_size);
		printf("\tMax kernel local group size:\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_DEVICE_MAX_CLOCK_FREQUENCY,sizeof(cl_uint),&entries,&ret_size);
		printf("\tMax Frequency (Mhz):\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,sizeof(cl_uint),&entries,&ret_size);
		printf("\tCache Line (bytes):\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_DEVICE_GLOBAL_MEM_SIZE,sizeof(cl_ulong),&long_entries,&ret_size);
		printf("\tGlobal Memory (MB):\t%llu\n",long_entries/1024/1024);

		clGetDeviceInfo(device_id,CL_DEVICE_LOCAL_MEM_SIZE,sizeof(cl_ulong),&long_entries,&ret_size);
		printf("\tLocal Memory (MB):\t%llu\n",long_entries/1024/1024);

		clGetDeviceInfo(device_id,CL_DEVICE_LOCAL_MEM_TYPE,sizeof(cl_device_local_mem_type),&mem_type,&ret_size);
		if(mem_type & CL_LOCAL)
			printf("\tLocal Memory Type:\tCL_LOCAL\n");
		else if(mem_type & CL_GLOBAL)
			printf("\tLocal Memory Type:\tCL_GLOBAL\n");
		else
			printf("\tLocal Memory Type:\tUNKNOWN\n");


		clGetDeviceInfo(device_id,CL_DEVICE_MAX_MEM_ALLOC_SIZE,sizeof(cl_ulong),&long_entries,&ret_size);
		printf("\tMax Mem Alloc (MB):\t%llu\n",long_entries/1024/1024);

		clGetDeviceInfo(device_id,CL_DEVICE_MAX_PARAMETER_SIZE,sizeof(size_t),&p_size,&ret_size);
		printf("\tMax Param Size (MB):\t%d\n",p_size);

		clGetDeviceInfo(device_id,CL_DEVICE_MEM_BASE_ADDR_ALIGN,sizeof(cl_uint),&entries,&ret_size);
		printf("\tBase Mem Align (bits):\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_DEVICE_ADDRESS_BITS,sizeof(cl_uint),&entries,&ret_size);
		printf("\tAddress Space (bits):\t%d\n",entries);

		clGetDeviceInfo(device_id,CL_DEVICE_IMAGE_SUPPORT,sizeof(cl_bool),&bool_entries,&ret_size);
		printf("\tImage Support:\t\t%d\n",bool_entries);

		clGetDeviceInfo(device_id,CL_DEVICE_TYPE,sizeof(fp_conf),&fp_conf,&ret_size);
		printf("\tFloat Functionality:\t");
		if(fp_conf & CL_FP_DENORM)
			printf("DENORM support ");
		if(fp_conf & CL_FP_ROUND_TO_NEAREST)
			printf("Round to nearest support ");
		if(fp_conf & CL_FP_ROUND_TO_ZERO)
			printf("Round to zero support ");
		if(fp_conf & CL_FP_ROUND_TO_INF)
			printf("Round to +ve/-ve infinity support ");
		if(fp_conf & CL_FP_FMA)
			printf("IEEE754 fused-multiply-add support ");
		if(fp_conf & CL_FP_INF_NAN)
			printf("INF and NaN support ");
		printf("\n");


		clGetDeviceInfo(device_id,CL_DEVICE_ERROR_CORRECTION_SUPPORT,sizeof(cl_bool),&bool_entries,&ret_size);
		printf("\tECC Support:\t\t%d\n",bool_entries);

		clGetDeviceInfo(device_id,CL_DEVICE_EXECUTION_CAPABILITIES,sizeof(cl_device_exec_capabilities),&exec_cap,&ret_size);
		printf("\tExec Functionality:\t");
		if(exec_cap & CL_EXEC_KERNEL)
			printf("CL_EXEC_KERNEL ");
		if(exec_cap & CL_EXEC_NATIVE_KERNEL)
			printf("CL_EXEC_NATIVE_KERNEL ");
		printf("\n");

		clGetDeviceInfo(device_id,CL_DEVICE_ENDIAN_LITTLE,sizeof(cl_bool),&bool_entries,&ret_size);
		printf("\tLittle Endian Device:\t%d\n",bool_entries);

		clGetDeviceInfo(device_id,CL_DEVICE_PROFILING_TIMER_RESOLUTION,sizeof(size_t),&p_size,&ret_size);
		printf("\tProfiling Res (ns):\t%d\n",p_size);

		clGetDeviceInfo(device_id,CL_DEVICE_AVAILABLE,sizeof(cl_bool),&bool_entries,&ret_size);
		printf("\tDevice Available:\t%d\n",bool_entries);

}

int OpenCLManager::isExtensionSupported(const char* support_str, const char* ext_string, size_t ext_buffer_size)
	{
		size_t offset = 0;
		const char* space_substr = strstr(ext_string + offset, " ");
		size_t space_pos = space_substr ? space_substr - ext_string : 0;
		while (space_pos < ext_buffer_size)
			{
				if( strncmp(support_str, ext_string + offset, space_pos) == 0 )
					{
						// Device supports requested extension!
							printf("Info: Found extension support ‘%s’!\n", support_str);
						return 1;
						}
				// Keep searching -- skip to next token string
					offset = space_pos + 1;
				space_substr = strstr(ext_string + offset, " ");
				space_pos = space_substr ? space_substr - ext_string : 0;
				}
		printf("Warning: Extension not supported ‘%s’!\n", support_str);
		return 0;
	}

#pragma package(smart_init)
