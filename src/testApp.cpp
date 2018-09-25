#include "testApp.h"


void testApp::setup(){ //NOTE: most of important settings are done here
    ofSetVerticalSync(false);
    ofSetFrameRate(120);
    ofBackground(ofColor::black);
    ofSetWindowShape(1200, 900);
    //ofToggleFullscreen();
    
    //init buffers
    for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i)   eeg_v[i].reserve(0);
    for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i)   eeg_raw_v[i].reserve(0);
    for(unsigned int i = 0; i < DEF_SERIAL_PACKET_AXIS_TOTAL; ++i) accel_v[i].reserve(0);
    for(unsigned int i = 0; i < DEF_SERIAL_PACKET_AXIS_TOTAL; ++i) eeg_pca[i].reserve(0);
    for(unsigned int i = 0; i < 3; ++i) eegByteArray_[i] = 0;
    for(unsigned int i = 0; i < 2; ++i) accelByteArray_[i] = 0;

    //init channels and serials
    startedStreaming = false;
    isReady = false;
    initSequenceCount = 0;
    numRequiredBytes =  DEF_SERIAL_PACKET_CH_SIZE * 3 + DEF_SERIAL_PACKET_AXIS_TOTAL * 2;
    readState = STATE_EOP;
    
    //set data source
    if(bReadFromFile){
        file.open("save.txt",ofFile::ReadOnly); //PARAM: when reading from a file saved from this software
        if(!file.exists()){
            ofLogError("The file " + file.path() + " is missing");
        }
        readBuffer.set(file);
        isReady = true;
        t_sample_read = ofGetSystemTime();
    }
    else{
        _serial.setup(DEF_SERIAL_PORT, DEF_SERIAL_BAUDRATE);
        _serial.flush();
        
        ofSleepMillis(400);
        _serial.writeByte(DEF_SENSOR_RESET);
        ofSleepMillis(100);
        _serial.writeByte(DEF_SENSOR_STREAMING_STOP); //TODO: verify
        if(DEF_SERIAL_PACKET_CH_TOTAL > 8){
            ofSleepMillis(100);
            _serial.writeByte('C');
        }
        else{
            ofSleepMillis(100);
            _serial.writeByte('c');
        }
    }
    
    //set frequency depending on the num of channel
    freq = 250;
    if(DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE) freq = 125;
    
    //value scale
    scale = 4.5 * 1000000 / 24.0 / (pow(2, 23) - 1.0); //PARAM: for default OpenBCI setting -- x24 gain, 4.5v power
    
    //filter index
    filter_index.push_back(DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE);
    filter_index.push_back(2 + (DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE));
    
    //set IIR filter coefficients //16ch needs 125hz numbers
    for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i ++){ //PARAM: add your filter if needed
        IIR notch50_250(FILTER_MAX_WINDOW, "notch 50hz(250hz)");
        notch50_250.setCoefficients({ -1.21449347931898, 2.29780334191380, -1.17207162934771, 0.931381682126901 }, { 0.965080986344734, -1.19328255433335, 2.29902305135123, -1.19328255433335, 0.965080986344734 });
        
        IIR notch50_125(FILTER_MAX_WINDOW, "notch 50hz(125hz)");
        notch50_125.setCoefficients({ 3.12516981877757, 4.30259605835520, 2.91046404408562, 0.867472133791670 }, { 0.931378858122983, 3.01781693143160, 4.30731047590091, 3.01781693143160, 0.931378858122983 });
        
        IIR bp5_50_250(FILTER_MAX_WINDOW, "bandpass 5-50hz (250hz)");
        bp5_50_250.setCoefficients({ -2.29905535603850, 1.96749775998445, -0.874805556449481, 0.219653983913695 }, { 0.175087643672101, 0, -0.350175287344202, 0, 0.175087643672101 });
        
        IIR bp5_40_250(FILTER_MAX_WINDOW, "bandpass 5-40hz (250hz)");
        bp5_40_250.setCoefficients({ -2.54633,   2.82051,  -1.59488,   0.41280 }, { 0.06746,   0.00000,  -0.13491,   0.00000,   0.06746 });
        
        IIR bp5_50_125(FILTER_MAX_WINDOW, "bandpass 5-50hz (125hz)");
        bp5_50_125.setCoefficients({ -0.517003774490767, -0.734318454224823, 0.103843398397761, 0.294636527587914 }, { 0.529967227069348, 0, -1.05993445413870, 0, 0.529967227069348 });
        
        IIR bp7_13_250(FILTER_MAX_WINDOW, "bandpass 7-13hz (250hz)");
        bp7_13_250.setCoefficients({ -3.67889546976404, 5.17970041352212, -3.30580189001670, 0.807949591420914 }, { 0.00512926836610803, 0, -0.0102585367322161, 0, 0.00512926836610803 });
        
        IIR bp7_13_125(FILTER_MAX_WINDOW, "bandpass 7-13hz (125hz)");
        bp7_13_125.setCoefficients({ -3.17162467236842, 4.11670870329067, -2.55619949640702, 0.652837763407545 }, { 0.0186503962278349, 0, -0.0373007924556699, 0, 0.0186503962278349 });
        
        IIR bp1_50_250(FILTER_MAX_WINDOW, "bandpass 1-50hz (250hz)");
        bp1_50_250.setCoefficients({ -2.35593463113158, 1.94125708865521, -0.784706375533419, 0.199907605296834 }, { 0.200138725658073, 0, -0.400277451316145, 0, 0.200138725658073 });
        
        IIR bp2_45_250(FILTER_MAX_WINDOW, "bandpass 2-45hz (250hz)");
        bp2_45_250.setCoefficients({ -2.49871,   2.27910,  -1.01022,   0.23162 }, { 0.16297,   0.00000,  -0.32595,   0.00000,   0.16297 });
        
        IIR bp1_50_125(FILTER_MAX_WINDOW, "bandpass 1-50hz (125hz)");
        bp1_50_125.setCoefficients({ -0.789307541613509, -0.853263915766877, 0.263710995896442, 0.385190413112446 }, { 0.615877232553135, 0, -1.23175446510627, 0, 0.615877232553135 });
        
        //60hz (and file)
        IIR notch60_250(FILTER_MAX_WINDOW, "notch 60hz(250hz)");
        notch60_250.setCoefficients({ -1.21449347931898, 2.29780334191380, -1.17207162934771, 0.931381682126901 }, { 0.965080986344734, -1.19328255433335, 2.29902305135123, -1.19328255433335, 0.965080986344734 });
        
        IIR notch60_125(FILTER_MAX_WINDOW, "notch 60hz(125hz)");
        notch60_125.setCoefficients({ -0.246778261129785, 1.94417178469135, -0.238158379221743, 0.931381682126902 }, { 0.965080986344733, -0.242468320175764, 1.94539149412878, -0.242468320175764, 0.965080986344733 });
        
        if(bReadFromFile){
            filters[i].push_back(notch60_250);
            filters[i].push_back(notch60_125);
        }
        else{ //PARAM: IMPORTANT - use 60hz filter if your country uses 60hz power source
            filters[i].push_back(notch50_250);
            filters[i].push_back(notch50_125);
        }
        //PARAM: add filters you need as needed. always put both 250hz and 125hz versions side-by-side in case you use 16ch setup
        filters[i].push_back(bp5_50_250);
        filters[i].push_back(bp5_50_125);
        filters[i].push_back(bp7_13_250);
        filters[i].push_back(bp7_13_125);
        filters[i].push_back(bp2_45_250);
        filters[i].push_back(bp1_50_125);
    }
    
    //fft
    for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i ++){
        fft[i].setFFTWindowSize(TRANSFORM_WINDOW);
        fft[i].setHopSize(1);
        fft[i].setComputePhase(false);
        fft[i].setComputeMagnitude(true);
        fill(fftBuffer[i].begin(), fftBuffer[i].end(), 0.);
        fft_active[i] = true;
        fftBuffer[i].resize(TRANSFORM_WINDOW);
        fftPCA[i].setFFTWindowSize(TRANSFORM_WINDOW);
        fftPCA[i].setHopSize(1);
        fftPCA[i].setComputePhase(false);
        fftPCA[i].setComputeMagnitude(true);
        fftPCABuffer[i].resize(TRANSFORM_WINDOW);
        fill(fftPCABuffer[i].begin(), fftPCABuffer[i].end(), 0.);
    }
    maxlog = log(1000000);
    minlog = log(100);
    fftSmooth = 0.9;
    
    //Haar
    nSweeps = TRANSFORM_WINDOW;
    for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i ++){
        haar[i].resize(TRANSFORM_WINDOW);
        fill(haar[i].begin(), haar[i].end(), 0.);
    }
    
    //classification
    svm_ts.setKernelType(SVM::LINEAR_KERNEL);
    svm_ts.enableScaling(true);
    svm_fft.setKernelType(SVM::LINEAR_KERNEL);
    svm_fft.enableScaling(true);
    trainingLabel = 0;
    classifiedLabel = -1;
    trainingData.setNumDimensions(DEF_SERIAL_PACKET_CH_TOTAL * TRANSFORM_WINDOW);
    
    //TODO: fft features -- not used not
    for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i ++){ //TODO: multidimensional FFT objects
        fftFeatures[i].init(fft[i].getFFTWindowSize(), 1, true, true, true, true, 10);
    }
}

//--------------------------------------------------------------
void testApp::update(){
    if(bReadFromFile){
        if(enabled){
            if(!readBuffer.isLastLine()){
                string s = readBuffer.getNextLine();
                if(s[0] == '%'){
                    cout << s << endl;
                }
                buffer.clear(); //reset for the new packet
                
                readData = ofSplitString(s, ", ");
                
                if(readData.size() > 0){
                    counter = (unsigned char)ofToInt(readData[0]);
                }
                
                if(ofGetSystemTime() - t_sample_read > 4){
                    t_sample_read = ofGetSystemTime();
                }
                else{
                    return;
                }
                
                if(readData.size() > DEF_SERIAL_PACKET_CH_TOTAL + 1){
                    int cnt = 0;
                    for(int i = 1; i < readData.size(); i++){
                        if(readData[i].empty()){
                            continue;
                        }
                        else{
                            int dat = ofToInt(readData[i]);
                            int absdat = abs(dat);
                            
                            unsigned char b1 = (unsigned char)(dat>>16);
                            unsigned char b2 = (unsigned char)(dat>>8);
                            unsigned char b3  = dat & 0xff;
                            
                            if(!signDirect && dat < 0){
                                b1 |= 0x80;
                            }
                            
                            buffer.push_back(b1);
                            buffer.push_back(b2);
                            buffer.push_back(b3);
                            
                            //cout << dat << " ";
                        }
                        cnt ++;
                        if(cnt >= DEF_SERIAL_PACKET_CH_TOTAL){
                            for(int j = 0; j < 6; j++) buffer.push_back((unsigned char)1);
                            startedStreaming = true;
                            processFrames();
                            break;
                        }
                    }
                    //cout << endl;
                }
            }
        }
    }
    else{
        while(_serial.available() > 0){
            if(!startedStreaming){
                read = _serial.readByte();
                
                if(!isReady){
                    cout << read;
                    if(read == '$'){
                        initSequenceCount++;
                    }
                    else{
                        initSequenceCount = 0;
                    }
                    if(initSequenceCount >= 3){
                        isReady = true;
                        cout << endl;
                    }
                }
                else if(read == DEF_SERIAL_PACKET_PREFIX){ //check the first PACKET message
                    startedStreaming = true;
                    
                    counter = _serial.readByte(); //framenum
                    buffer.clear(); //reset for the new packet
                    flagByte = 0; //for EOP and PREFIX
                    readState = STATE_READING;
                }
            }
            else{
                if(readState == STATE_EOP){ //same thing as above
                    counter = _serial.readByte();
                    prev_buffer = buffer; //TODO: remove for actual run
                    buffer.clear();
                    flagByte = 0;
                    readState = STATE_READING;
                }
                
                if(readState == STATE_READING){
                    read = _serial.readByte();
                    
                    if(flagByte == DEF_SERIAL_PACKET_EOP){
                        if(read == DEF_SERIAL_PACKET_PREFIX){
                            processFrames();
                            readState = STATE_EOP;
                            continue;
                        }
                        else if(read == DEF_SERIAL_PACKET_EOP){
                            buffer.push_back(flagByte);
                        }
                        else{
                            buffer.push_back(flagByte);
                            buffer.push_back(read);
                            flagByte = 0; //meaning the previous EOP was just data
                        }
                    }
                    else{ //flagbyte should be 0 by default
                        if(read == DEF_SERIAL_PACKET_EOP && buffer.size() > numRequiredBytes - 2){
                            flagByte = read;
                        }
                        else{ //regular data
                            buffer.push_back(read);
                        }
                    }
                }
            }
        }
    }
    
    if(bSamplingTimeSeries){ //TODO: move it to the processData function
        bSamplingTimeSeriesDone = false;
        
        if(eeg_v[0].size() >= SAMPLING_WINDOW){
            
            MatrixDouble data(SAMPLING_WINDOW, DEF_SERIAL_PACKET_CH_TOTAL);
            
            //save data
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                for(int j = 0; j < SAMPLING_WINDOW; j ++){
                    data[j][i] = eeg_v[i][eeg_v[i].size() - SAMPLING_WINDOW + j];
                }
            }
        
            if( !pcaTimeSeries.computeFeatureVector( data, 0.99 ) ){ // TODO: second argument?
                cout << "ERROR: Failed to compute feature vector!\n";
                return EXIT_FAILURE;
            }
            cout << "PCA on time-series done" << endl;
            
            //Get the number of principal components
            UINT numPrincipalComponents = pcaTimeSeries.getNumPrincipalComponents();
            cout << "Number of Principal Components: " << numPrincipalComponents << endl;
            
            bSamplingTimeSeriesDone = true;
            bSamplingTimeSeries = false;
            view_mode = 4; //TODO: for easy experiments
        }
    }
    
    /*
    else if(bSamplingFFT){ //TODO: doesn't seem to work now
        bSamplingFFTDone = false;
        
        if(fft_v[0].size() >= SAMPLING_WINDOW){
            
            MatrixDouble data(DEF_SERIAL_PACKET_CH_TOTAL * SAMPLING_WINDOW, TRANSFORM_WINDOW);
            
            //save data
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                for(int j = 0; j < SAMPLING_WINDOW; j ++){
                    int row = i * SAMPLING_WINDOW + j;
                    for(int k = 0; k < TRANSFORM_WINDOW; k ++){
                        data[row][k] = 0.0001 * fft_v[i][fft_v[i].size() - SAMPLING_WINDOW + j][k];
                    }
                }
            }
            
            if( !pcaFFT.computeFeatureVector( data, 0.96 ) ){ // TODO: second argument?
                cout << "ERROR: Failed to compute feature vector!\n";
                return EXIT_FAILURE;
            }
            cout << "PCA on FFT done" << endl;
            
            //Get the number of principal components
            UINT numPrincipalComponents = pcaFFT.getNumPrincipalComponents();
            cout << "Number of Principal Components: " << numPrincipalComponents << endl;
            
            bSamplingFFTDone = true;
            bSamplingFFT = false;
        }
    }
    */
    
    if(bSampleRecording){ //record training sample
        long long t_s = ofGetSystemTime() - t_sampling_start;
        
        if(trainingLabel >= 0){
            if(t_s > t_sampling_delay + t_sampling_duration){
                bSampleRecording = false;
                trainingLabel++;
            }
            else if(t_s > t_sampling_delay){
                VectorDouble v;
                for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i ++){
                    for(int j = 0; j < TRANSFORM_WINDOW; j ++){
                        if(bSamplingFFTDone){
                            v.push_back( fftPCA[i].getFeatureVector()[j] );
                        }
                        else{
                            v.push_back( fft[i].getFeatureVector()[j] );
                        }
                    }
                }
                trainingData.addSample(trainingLabel, v);
            }
        }
        else{
            bSampleRecording = false;
        }
    }
    
    if(bTrained){ //if trained, run classification on the data
        vector<double> v;
        for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i ++){
            for(int j = 0; j < TRANSFORM_WINDOW; j ++){
                if(bSamplingFFTDone){
                    v.push_back( fftPCA[i].getFeatureVector()[j] );
                }
                else{
                    v.push_back( fft[i].getFeatureVector()[j] );
                }
            }
        }
        svm_fft.predict(v);
        classifiedLabel = svm_fft.getPredictedClassLabel();
    }
}

//--------------------------------------------------------------
void testApp::draw(){
    //plots
    if(accel_v[0].size() > 0) //make sure we have enough frame
    {
        if(view_mode == 0){ //1-8 time-series
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_SIZE; ++i){
                ofPushMatrix();
                ofTranslate(20, 100 * (i + 1));
                ofSetColor(255);
                ofDrawLine(0, 0, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, 0);
                ofSetColor(0, 255, 255);
                for(int j = 1; j < eeg_v[i].size() - 1; j++){
                    ofDrawLine( (j-1) * PLOT_SCALE_X, eeg_v[i][j-1] * PLOT_SCALE_Y,
                               j * PLOT_SCALE_X, eeg_v[i][j] * PLOT_SCALE_Y);
                }
                ofPopMatrix();
            }
        }
        else if(view_mode == 1){ //9-16 time-series
            if(DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE){
                for(unsigned int i = DEF_SERIAL_PACKET_CH_SIZE; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                    ofPushMatrix();
                    ofTranslate(20, 100 * (i + 1 - DEF_SERIAL_PACKET_CH_SIZE ));
                    ofSetColor(255);
                    ofDrawLine(0, 0, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, 0);
                    ofSetColor(0, 255, 255);
                    for(int j = 1; j < eeg_v[i].size() - 1; j++){
                        ofDrawLine( (j-1) * PLOT_SCALE_X, eeg_v[i][j-1] * PLOT_SCALE_Y,
                                   j * PLOT_SCALE_X, eeg_v[i][j] * PLOT_SCALE_Y);
                    }
                    ofPopMatrix();
                }
            }
        }
        else if(view_mode == 2){ //fft
            ofPushMatrix();
            ofTranslate(20, ofGetWindowHeight() - 200);
            
            ofSetColor(255);
            ofDrawLine(0, 0, TRANSFORM_WINDOW * 2 * PLOT_SCALE_X_FFT, 0);
            ofDrawLine(TRANSFORM_WINDOW *  PLOT_SCALE_X_FFT, 0, TRANSFORM_WINDOW *  PLOT_SCALE_X_FFT, -500);
            
            for(unsigned int i = 0; i < num_bands; i++){
                int x = TRANSFORM_WINDOW *  PLOT_SCALE_X_FFT * processing_band_high_Hz[i] * 2 / freq;
                ofDrawLine(x, 0, x, -300);
            }
            
            VectorDouble frequencyBins = fft[0].getFrequencyBins(freq/2);
            int nBin = frequencyBins.size();
            for(unsigned int i = 0; i < nBin; i+=16){
                int x = PLOT_SCALE_X_FFT * i - 10;
                ofDrawBitmapString(frequencyBins[i], x, 20);
            }

            if(!bSamplingFFTDone){ //not done
                for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                    if(fft_active[i]){
                        for(int j = 1; j < fftBuffer[i].size(); j++){
                            double a = (log( abs(fftBuffer[i][j-1])) - minlog) / (maxlog - minlog);
                            double b = ( log( abs(fftBuffer[i][j])) - minlog) / (maxlog - minlog);
                            
                            //TODO: this log scale thing needs some more work
                            a = abs(fftBuffer[i][j-1]) * 0.0003;
                            b = abs(fftBuffer[i][j]) * 0.0003;
                            
                            ofDrawLine( (j-1) * 2 * PLOT_SCALE_X_FFT, -a * PLOT_SCALE_Y_LOG,
                                       j * 2 * PLOT_SCALE_X_FFT, -b * PLOT_SCALE_Y_LOG);
                        }
                    }
                }
            }
            else{ //TODO: move to a speerate view mode //TODO: process FFT_PCA in the update thread
                
                for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                    if(fft_active[i]){
                        
                        MatrixDouble data(1, TRANSFORM_WINDOW);
                        
                        for(int j = 0; j < TRANSFORM_WINDOW; j++){
                            data[0][j] = fftBuffer[i][j];
                        }
                        
                        MatrixDouble prjData;
                        if( !pcaFFT.project( data, prjData ) ){
                            cout << "ERROR: Failed to project data!\n";
                            return EXIT_FAILURE;
                        }
                        
                        for(int j = 1; j < TRANSFORM_WINDOW; j++){
                            double a = (log( abs(prjData[0][j - 1])) - minlog) / (maxlog - minlog);
                            double b = ( log( abs(prjData[0][j])) - minlog) / (maxlog - minlog);
                            
                            //TODO: this log scale thing needs some more work
                            a = abs(prjData[0][j - 1]) * 0.0003;
                            b = abs(prjData[0][j]) * 0.0003;
                            
                            ofDrawLine( (j-1) * 2 * PLOT_SCALE_X_FFT, -a * PLOT_SCALE_Y_LOG,
                                       j * 2 * PLOT_SCALE_X_FFT, -b * PLOT_SCALE_Y_LOG);
                            
                        }
                    }
                }
            }
            ofPopMatrix();
        }
        else if(view_mode == 3){ //haar
            ofPushMatrix();
            ofTranslate(20, ofGetWindowHeight() - 200);
            
            ofSetColor(255);
            ofDrawLine(0, 0, TRANSFORM_WINDOW * PLOT_SCALE_X_FFT, 0);
            
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                if(fft_active[i]){
                    
                    ofPushMatrix();
                    ofTranslate(0, -200);
                    ofSetColor(255);
                    ofDrawLine(0, 0, TRANSFORM_WINDOW * PLOT_SCALE_X_FFT, 0);
                    ofSetColor(0, 255, 255);
                    if(eeg_v[i].size() >= TRANSFORM_WINDOW){
                        for(int j = 1; j < TRANSFORM_WINDOW; j++){
                            ofDrawLine( (j-1) * PLOT_SCALE_X_FFT, eeg_v[i][eeg_v[i].size() - TRANSFORM_WINDOW + j - 1] * PLOT_SCALE_Y,
                                       j * PLOT_SCALE_X_FFT, eeg_v[i][eeg_v[i].size() - TRANSFORM_WINDOW + j] * PLOT_SCALE_Y);
                        }
                    }
                    ofPopMatrix();
                    
                    int n = haar[i].size();
                    //cout << n << endl;
                    int START_INDEX = n / 4;
                    int NUM_FREQS = 2;
                    int a = 0;
                    double b = 0;
                    while (START_INDEX > 1) {
                        int ODD = 1;
                        for (int K = 0; K < NUM_FREQS; K++) {
                            
                            ofDrawLine( (a-1) * PLOT_SCALE_X_FFT, -b,
                                           a * PLOT_SCALE_X_FFT, -haar[i][START_INDEX * ODD]);
                            
                            a++;
                            b = haar[i][START_INDEX * ODD];
                            ODD += 2;
                        }
                        START_INDEX /= 2;
                        NUM_FREQS *= 2;
                    }
                }
            }
            ofPopMatrix();
        }
        if(view_mode == 4){ //1-8 time-series -- after PCA
            if(eeg_pca[0].size() > 0){
                for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_SIZE; ++i){
                    ofPushMatrix();
                    ofTranslate(20, 100 * (i + 1));
                    ofSetColor(255);
                    ofDrawLine(0, 0, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, 0);
                    ofSetColor(0, 255, 255);
                    for(int j = 1; j < eeg_pca[i].size() - 1; j++){
                        ofDrawLine( (j-1) * PLOT_SCALE_X, eeg_pca[i][j-1] * PLOT_SCALE_Y,
                                   j * PLOT_SCALE_X, eeg_pca[i][j] * PLOT_SCALE_Y);
                    }
                    ofPopMatrix();
                }
            }
        }
        else if(view_mode == 5){ //TODO: 9-16 time-series -- after PCA
            if(DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE){
                for(unsigned int i = DEF_SERIAL_PACKET_CH_SIZE; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                }
            }
        }
        
        else if(view_mode == 6){ //fft after PCA
            ofPushMatrix();
            ofTranslate(20, ofGetWindowHeight() - 200);
            
            ofSetColor(255);
            ofDrawLine(0, 0, TRANSFORM_WINDOW * 2 * PLOT_SCALE_X_FFT, 0);
            ofDrawLine(TRANSFORM_WINDOW *  PLOT_SCALE_X_FFT, 0, TRANSFORM_WINDOW *  PLOT_SCALE_X_FFT, -500);
            
            for(unsigned int i = 0; i < num_bands; i++){
                int x = TRANSFORM_WINDOW *  PLOT_SCALE_X_FFT * processing_band_high_Hz[i] * 2 / freq;
                ofDrawLine(x, 0, x, -300);
            }
            
            VectorDouble frequencyBins = fft[0].getFrequencyBins(freq/2);
            int nBin = frequencyBins.size();
            
            for(unsigned int i = 0; i < nBin; i+=16){
                int x = PLOT_SCALE_X_FFT * i - 10;
                ofDrawBitmapString(frequencyBins[i], x, 20);
            }
            
            if(bSamplingTimeSeriesDone){ //not done
                for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                    if(fft_active[i]){
                        for(int j = 1; j < fftPCABuffer[i].size(); j++){
                            double a = (log( abs(fftPCABuffer[i][j-1])) - minlog) / (maxlog - minlog);
                            double b = ( log( abs(fftPCABuffer[i][j])) - minlog) / (maxlog - minlog);
                            
                            //TODO: this log scale thing needs some more work
                            a = abs(fftPCABuffer[i][j-1]) * 0.0003;
                            b = abs(fftPCABuffer[i][j]) * 0.0003;
                            
                            ofDrawLine( (j-1) * 2 * PLOT_SCALE_X_FFT, -a * PLOT_SCALE_Y_LOG,
                                       j * 2 * PLOT_SCALE_X_FFT, -b * PLOT_SCALE_Y_LOG);
                        }
                    }
                }
            }
            ofPopMatrix();
        }

        else if(view_mode == 7 ){ // show classification results
            
            if(bTrained){
                //update classification result
                if(ofGetSystemTime() - t_class_likelihood >= 50){
                    t_class_likelihood = ofGetSystemTime();
                    
                    float likelihood = 0.f;
                    
                    for(int i = 0; i < svm_fft.getNumClasses() && i < NUM_MAX_CLASSES; i++){
                        
                        float dat = svm_fft.getClassLikelihoods()[i];
                        lh[i].push_back(dat);
                        while(lh[i].size() > NUM_MAINTAINED_SAMPLES) lh[i].erase(lh[i].begin());
                        
                        if(dat > likelihood){
                            likelihood = dat;
                            classifiedLabel = i;
                        }
                        
                        if(bPrintLikelihood){
                            cout << svm_fft.getClassLikelihoods()[i] ;
                            if( i < svm_fft.getNumClasses() - 1  && i < NUM_MAX_CLASSES - 1 ) cout <<",";
                        }
                    }
                    if(bPrintLikelihood) cout << endl;
                    
                    //do statistics on labels
                    labelRaw.push_back(classifiedLabel);
                    while(labelRaw.size() > NUM_MAINTAINED_SAMPLES) labelRaw.erase(labelRaw.begin());
                    
                    float total = LABELING_WINDOW;
                    if(labelRaw.size() < LABELING_WINDOW) total = labelRaw.size();
                    if(labelRaw.size() > 0){
                        int count[NUM_MAX_CLASSES];
                        for(unsigned int i = 0; i < NUM_MAX_CLASSES; ++i){
                            count[i] = 0;
                        }
                        
                        for(int j = labelRaw.size() - total; j < labelRaw.size(); j++){
                            if(labelRaw[j] >= 0 && labelRaw[j] < NUM_MAX_CLASSES){
                                count[labelRaw[j]]++;
                            }
                        }
                        
                        int newFilteredLabel = 0, maxCount = 0;
                        for(unsigned int i = 0; i < NUM_MAX_CLASSES; ++i){
                            //cout << i << " - " << count[i]<< endl;
                            if(count[i] > maxCount){
                                newFilteredLabel = i;
                                maxCount = count[i];
                            }
                            
                            lh_ratio[i].push_back(count[i]/total);
                            while(lh_ratio[i].size() > NUM_MAINTAINED_SAMPLES) lh_ratio[i].erase(lh_ratio[i].begin());
                        }
                        
                        labelFiltered.push_back(newFilteredLabel);
                        while(labelFiltered.size() > NUM_MAINTAINED_SAMPLES) labelFiltered.erase(labelFiltered.begin());
                    }
                }
                
                ofSetLineWidth(2);
                
                //likelihood
                ofPushMatrix();
                ofTranslate(20, 220);
                ofSetColor(155);
                ofDrawLine(0, -PLOT_SCALE_Y_LIKELIHOOD, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, -PLOT_SCALE_Y_LIKELIHOOD);
                ofDrawLine(0, 0, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, 0);
                for(unsigned int i = 0; i < svm_fft.getNumClasses() && i < NUM_MAX_CLASSES; ++i){
                    ofSetColor(255*(i%3 == 0), 255*(i%3 == 1), 255*(i%3 == 2));
                    if(lh[i].size() > 1){
                        for(int j = 1; j < lh[i].size() - 1; j++){
                            ofDrawLine( (j-1) * PLOT_SCALE_X, -lh[i][j-1] * PLOT_SCALE_Y_LIKELIHOOD,
                                       j * PLOT_SCALE_X, -lh[i][j] * PLOT_SCALE_Y_LIKELIHOOD);
                        }
                    }
                }
                ofPopMatrix();
                
                //get stats
                ofPushMatrix();
                ofTranslate(20, 440);
                ofSetColor(155);
                ofDrawLine(0, 0, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, 0);
                ofDrawLine(0, -PLOT_SCALE_Y_LIKELIHOOD, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, -PLOT_SCALE_Y_LIKELIHOOD);
                for(unsigned int i = 0; i < svm_fft.getNumClasses() && i < NUM_MAX_CLASSES; ++i){
                    ofSetColor(255*(i%3 == 0), 255*(i%3 == 1), 255*(i%3 == 2));
                    if(lh_ratio[i].size() > 1){
                        for(int j = 1; j < lh_ratio[i].size() - 1; j++){
                            ofDrawLine( (j-1) * PLOT_SCALE_X, -lh_ratio[i][j-1] * PLOT_SCALE_Y_LIKELIHOOD,
                                       j * PLOT_SCALE_X, -lh_ratio[i][j] * PLOT_SCALE_Y_LIKELIHOOD);
                        }
                    }
                }
                ofPopMatrix();
                
                //label
                ofPushMatrix();
                ofTranslate(20, 660);
                ofSetColor(155);
                ofDrawLine(0, 0, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, 0);
                ofDrawLine(0, -PLOT_SCALE_Y_LIKELIHOOD, NUM_MAINTAINED_SAMPLES * PLOT_SCALE_X, -PLOT_SCALE_Y_LIKELIHOOD);
                ofSetColor(255);
                if(labelRaw.size() > 1){
                    for(int j = 1; j < labelRaw.size() - 1; j++){
                        ofDrawLine( (j-1) * PLOT_SCALE_X, -labelRaw[j-1] * PLOT_SCALE_Y_LIKELIHOOD * 0.3,
                                   j * PLOT_SCALE_X, -labelRaw[j] * PLOT_SCALE_Y_LIKELIHOOD * 0.3);
                    }
                }
                ofSetColor(0, 255, 255);
                if(labelFiltered.size() > 1){
                    for(int j = 1; j < labelFiltered.size() - 1; j++){
                        ofDrawLine( (j-1) * PLOT_SCALE_X, -labelFiltered[j-1] * PLOT_SCALE_Y * 0.3,
                                   j * PLOT_SCALE_X, -labelFiltered[j] * PLOT_SCALE_Y * 0.3);
                    }
                }
                ofPopMatrix();
                
                ofSetLineWidth(1);
            }
        }
    }
    
    if(bShowBandpower){ //visualize band power // TODO: move it to a seperate update process
        int band_id = 0;
        int band_cnt[] = {0, 0, 0, 0, 0};
        
        VectorDouble frequencyBins = fft[0].getFrequencyBins(freq/2);
        
        for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
            if(fft_active[i]){
                band_id = 0;
                for(int j = 0; j < TRANSFORM_WINDOW; j++){
                    if( band_id < num_bands -1 && frequencyBins[j] >= processing_band_low_Hz[band_id]){
                        band_id ++;
                    }
                    if(band_id == num_bands - 1 && frequencyBins[j] > processing_band_high_Hz[band_id]){
                        band_id++;
                    }
                    if(band_id == num_bands){
                        band_id = 0;
                        break;
                    }
                    
                    bandPower[band_id] += fftBuffer[i][j];
                    band_cnt[band_id]++;
                }
            }
        }
        ofPushMatrix();
        ofTranslate(500, 800);
        for(int i = 0; i < num_bands; i++){
            if(band_cnt[i] > 0){
                bandPower[i] /= (float)band_cnt[i];
                
                ofSetColor(0, 255, 0);
                ofDrawRectangle(i * 40, -bandPower[i]/4, 35, bandPower[i]/4);
            }
        }
        ofPopMatrix();
    }
    
    //rest of the status log
    if(isReady){
        if(bReadFromFile){
            ofSetColor(0, 255, 0);
            ofDrawBitmapString("Read File", ofGetWindowWidth() - 100, 20);
        }
        else {
            if(startedStreaming){
                if(enabled){
                    ofSetColor(255, 255, 255);
                    ofDrawBitmapString("Streaming", ofGetWindowWidth() - 100, 20);
                }
                else{
                    ofSetColor(155);
                    ofDrawBitmapString("Paused", ofGetWindowWidth() - 100, 20);
                }
            }
            else{
                ofSetColor(0, 255, 0);
                ofDrawBitmapString("Ready", ofGetWindowWidth() - 100, 20);
            }
        }
    }
    ofSetColor(155);
    ofDrawBitmapString(ofToString(ofGetFrameRate()) + " fps", ofGetWindowWidth() - 100, 40);
    
    //notch filter
    if(filter_index.size() > 0 && filter_index[0] >= 0){
        ofSetColor(0, 180, 180);
        ofDrawBitmapString(filters[0][filter_index[0]].getName(), ofGetWindowWidth() - 200, 60);
    }
    else{
        ofSetColor(155);
        ofDrawBitmapString("no notch filter", ofGetWindowWidth() - 160, 60);
    }
    
    //second filter - TODO: make it a generic number
    if(filter_index.size() > 1 && filter_index[1] >= 0){
        ofSetColor(0, 180, 180);
        ofDrawBitmapString(filters[0][filter_index[1]].getName(), ofGetWindowWidth() - 200, 80);
    }
    else{
        ofSetColor(155);
        ofDrawBitmapString("no band filter", ofGetWindowWidth() - 160, 80);
    }
    
    //show visualization info
    if(view_mode == 0){
        ofDrawBitmapString("CH 1-8", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 1){
        ofDrawBitmapString("CH 9-16", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 2){
        ofDrawBitmapString("FFT", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 3){
        ofDrawBitmapString("HAAR", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 4){
        ofDrawBitmapString("PCA ch 1-8", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 5){
        ofDrawBitmapString("PCA ch 9-16", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 6){
        ofDrawBitmapString("PCA FFT", ofGetWindowWidth() - 160, 100);
    }
    else if(view_mode == 7){
        ofDrawBitmapString("Classification", ofGetWindowWidth() - 160, 100);
    }
    
    ofSetColor(155, 155, 155);
    ofDrawBitmapString("training label: " + ofToString(trainingLabel), ofGetWindowWidth() - 160, 120);
    ofDrawBitmapString("# samples: " + ofToString(trainingData.getNumSamples()), ofGetWindowWidth() - 160, 140);
    if(bSampleRecording) ofSetColor(255, 0, 0);
    else if(bTrained) ofSetColor(0, 255, 0);
    ofDrawBitmapString("classified: " + ofToString(classifiedLabel), ofGetWindowWidth() - 160, 160);
    for(int i = 0; i < svm_fft.getNumClasses(); i++){
        ofDrawBitmapString("#" + ofToString(i) + ": " + ofToString(svm_fft.getClassLikelihoods()[i]), ofGetWindowWidth() - 160, 180 + i*20);
    }
    if(bPrintLikelihood){
        ofSetColor(255, 0, 0);
        ofDrawBitmapString("printing", ofGetWindowWidth() - 160, 180 + 20*svm_fft.getNumClasses());
    }

    //fft enabled flag
    ofSetColor(30);
    ofDrawRectangle(0, 0, DEF_SERIAL_PACKET_CH_TOTAL*30 + 40, 30);
    for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
        if(fft_active[i]) ofSetColor(0, 255, 0);
        else ofSetColor(155);
        ofDrawBitmapString(ofToString(i+1), 20 + i*30, 20);
    }
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){ //NOTE: all key shortcuts are defined here
    if(key == OF_KEY_LEFT){
        view_mode--;
        view_mode += NUM_VIEW_MODE;
        view_mode %= NUM_VIEW_MODE;
        if(DEF_SERIAL_PACKET_CH_TOTAL <= DEF_SERIAL_PACKET_CH_SIZE && view_mode == 1) view_mode--;
        if(DEF_SERIAL_PACKET_CH_TOTAL <= DEF_SERIAL_PACKET_CH_SIZE && view_mode == 5) view_mode--;
    }
    if(key == OF_KEY_RIGHT){
        view_mode++;
        view_mode %= NUM_VIEW_MODE;
        if(DEF_SERIAL_PACKET_CH_TOTAL <= DEF_SERIAL_PACKET_CH_SIZE && view_mode == 1) view_mode++;
        if(DEF_SERIAL_PACKET_CH_TOTAL <= DEF_SERIAL_PACKET_CH_SIZE && view_mode == 5) view_mode++;
    }
    //enable disable fft on each channel -- you don't really need to do anything with this
    if(key == '1'){
        int idx = 0;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '2'){
        int idx = 1;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '3'){
        int idx = 2;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '4'){
        int idx = 3;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '5'){
        int idx = 4;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '6'){
        int idx = 5;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '7'){
        int idx = 6;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '8'){
        int idx = 7;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '!'){
        int idx = 8;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '@'){
        int idx = 9;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '#'){
        int idx = 10;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '$'){
        int idx = 11;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '%'){
        int idx = 12;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '^'){
        int idx = 13;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '&'){
        int idx = 14;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == '*'){
        int idx = 15;
        if(idx < DEF_SERIAL_PACKET_CH_TOTAL) fft_active[idx] = !fft_active[idx];
    }
    if(key == 'x'){
        for(int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; i++){
            fft_active[i] = false;
        }
    }
    if(key == ' '){ //streaming start/pause
        if(isReady){
            enabled =! enabled;
            if(!bReadFromFile){
                if(enabled){
                    _serial.writeByte(DEF_SENSOR_STREAMING_START);
                }
                else{
                    _serial.writeByte(DEF_SENSOR_STREAMING_STOP);
                }
            }
        }
    }
    if(key == 'n'){ //enable/disable notch filter
        if(filter_index.size() > 0){
            if(filter_index[0] < 0) filter_index[0] = (int)(DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE);
            else filter_index[0] = -1;
        }
    }
    if(key == 'b'){ //cycle through bandpass filters
        if(filter_index.size() > 1){
            if(filter_index[1] < 0){
                filter_index[1] = 2 + (DEF_SERIAL_PACKET_CH_TOTAL > DEF_SERIAL_PACKET_CH_SIZE);
            }
            else if(filter_index[1] >= filters[0].size() - 2){
                filter_index[1] = -1;
            }
            else {
                filter_index[1] += 2;
            }
        }
    }
    if(key == 'r'){ //save samples
        if(bRecording){
            save.close();
            cout << "done." <<endl;
        }
        else{
            string fn = "save.txt"; //PARAM: change file path
            cout << "[saving to file " << fn << "..." <<endl;
            save.open(fn,ofFile::WriteOnly);
        }
        bRecording =! bRecording;
    }
    if(key == 'p'){ //show band power information -- not really helpful
        bShowBandpower = !bShowBandpower;
    }
    if(key == 't'){ //apply PCA on timeseries EEG dats
        bSamplingTimeSeries = !bSamplingTimeSeries;
    }
    if(key == 'f'){ //apply PCA on fft -- not working well now
        bSamplingFFT = !bSamplingFFT;
    }
    if(key == 's'){ // sample training data
        if(!bSampleRecording){
            t_sampling_start = ofGetSystemTime();
        }
        bSampleRecording = true;
    }
    if(key == 'd'){ // traing SVM
        if(trainingData.getNumSamples() > 0){
            cout << svm_fft.train( trainingData );
            bTrained = true;
            view_mode = 7; //TODO: for easy experiments
        }
    }
    if(key == ','){
        trainingLabel --;
        if(trainingLabel < 0) trainingLabel = 0;
    }
    if(key == '.'){
        trainingLabel++;
    }
    if(key == 'i'){ //reset training data when re-traning is required
        bTrained = false;
        trainingData.clear();
        trainingLabel = 0;
        classifiedLabel = -1;
        trainingData.setNumDimensions(DEF_SERIAL_PACKET_CH_TOTAL * TRANSFORM_WINDOW);
        svm_fft.clear();
        svm_ts.clear();
    }
    if(key == 'l'){ //NOTE: print likelihood data on console
        bPrintLikelihood = !bPrintLikelihood;
    }
    if(key == '`'){
        ofToggleFullscreen();
    }
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 
    
}

//--------------------------------------------------------------
void testApp::processFrames(){ //IMPORANT FUNCTION - but should work without much problem
    if(!startedStreaming) return;
    
    if(buffer.size() != numRequiredBytes){
        //TODO: very heuristic implementation... but works!
        if(buffer.size() == numRequiredBytes + 1 && 254 == buffer[0]){
            buffer.erase(buffer.begin());
        }
        else{
            //print in case it's wrong: shouldn't happen much (if not at all)
            cout << frameNum << "- " << (int)counter << ": wrong sized packet: " << buffer.size() << endl;
            /*
             cout <<"(" << DEF_SERIAL_PACKET_EOP <<" " <<DEF_SERIAL_PACKET_PREFIX <<")" <<endl;
             for(int i = 0; i < buffer.size(); i++){
             cout << (int)buffer[i] << " ";
             }
             cout <<endl << endl;
             cout <<"prev: " <<endl;
             for(int i = 0; i < prev_buffer.size(); i++){
             cout << (int)prev_buffer[i] << " ";
             }
             cout <<endl << endl;
             */
            return;
        }
    }
    
    //num of frames skipped
    int frameSkipped = 0;
    if(frameNum > counter){
        frameSkipped = 255 + counter - frameNum - 1;
    }
    else{
        frameSkipped = counter - frameNum - 1;
    }
    
    //add filler for lost frames
    if(frameSkipped > 0){
        for(int k = 0; k < frameSkipped; k++){
            
            if(bRecording){
                save << (frameNum + k) <<", ";
            }
            
            //alternate between
            if( DEF_SERIAL_PACKET_CH_TOTAL  ==  DEF_SERIAL_PACKET_CH_SIZE ||
               (frameNum + k)%2 == 0 ){ //lower channel -- DEF_SERIAL_PACKET_CH_SIZE
                
                for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_SIZE; ++i){
                    if(eeg_raw_v[i].size() > 0){ //only if we have sample
                        //get old value
                        double filteredValue = eeg_raw_v[i][eeg_raw_v[i].size() - 1];
                        
                        if(bRecording){
                            save << interpret24bitAsInt32(eegByteArray_) <<", ";
                        }
                        
                        //copy raw value
                        eeg_raw_v[i].push_back(filteredValue);
                        if(eeg_raw_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_raw_v[i].erase(eeg_raw_v[i].begin());
                        
                        //apply all the filters
                        for(int f = 0; f < filter_index.size(); f++){
                            if(filter_index[f] >= 0){
                                filteredValue = filters[i][filter_index[f]].filter(filteredValue);
                            }
                        }
                        
                        //fft
                        fft[i].update(filteredValue);
                        VectorDouble fb = fft[i].getFeatureVector();
                        for(int j = 0; j < fb.size(); j ++){
                            fftBuffer[i][j] = fftBuffer[i][j] * fftSmooth + fb[j] * (1.0 - fftSmooth);
                        }
                        
                        //add processed value
                        eeg_v[i].push_back(filteredValue);
                        if(eeg_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_v[i].erase(eeg_v[i].begin());
                        
                        //haar
                        if(eeg_v[i].size() >= TRANSFORM_WINDOW){
                            haar[i].clear();
                            for(int j = 0; j < TRANSFORM_WINDOW; j++){
                                haar[i].push_back(eeg_v[i][eeg_v[i].size() - TRANSFORM_WINDOW + j]);
                            }
                            //do the transform
                            inPlaceFastHaarWaveletTransform_nSweeps(haar[i], TRANSFORM_WINDOW, nSweeps);
                        }
                    }
                }
            }
            else{
                for(unsigned int i = DEF_SERIAL_PACKET_CH_SIZE; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                    if(eeg_raw_v[i].size() > 0){ //only if we have sample
                        //get old value
                        double filteredValue = eeg_raw_v[i][eeg_raw_v[i].size() - 1];
                        
                        if(bRecording){
                            save << interpret24bitAsInt32(eegByteArray_) <<", ";
                        }
                        
                        //copy raw value
                        eeg_raw_v[i].push_back(filteredValue);
                        if(eeg_raw_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_raw_v[i].erase(eeg_raw_v[i].begin());
                        
                        //apply all the filters
                        for(int f = 0; f < filter_index.size(); f++){
                            if(filter_index[f] >= 0){
                                filteredValue = filters[i][filter_index[f]].filter(filteredValue);
                            }
                        }
                        
                        //fft
                        fft[i].update(filteredValue);
                        VectorDouble fb = fft[i].getFeatureVector();
                        for(int j = 0; j < fb.size(); j ++){
                            fftBuffer[i][j] = fftBuffer[i][j] * fftSmooth + fb[j] * (1.0 - fftSmooth);
                        }
                        
                        //add processed value
                        eeg_v[i].push_back(filteredValue);
                        if(eeg_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_v[i].erase(eeg_v[i].begin());
                        
                        //haar
                        if(eeg_v[i].size() >= TRANSFORM_WINDOW){
                            haar[i].clear();
                            for(int j = 0; j < TRANSFORM_WINDOW; j++){
                                haar[i].push_back(eeg_v[i][eeg_v[i].size() - TRANSFORM_WINDOW + j]);
                            }
                            //do the transform
                            inPlaceFastHaarWaveletTransform_nSweeps(haar[i], TRANSFORM_WINDOW, nSweeps);
                        }
                    }
                }
            }
            
            //fill in missing ACCEL values
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_AXIS_TOTAL; ++i){
                if(accel_v[i].size() > 0){ //only if we have sample
                    accel_v[i].push_back(accel_v[i][accel_v[i].size() - 1]);
                    if(accel_v[i].size() > NUM_MAINTAINED_SAMPLES) accel_v[i].erase(accel_v[i].begin());
                    
                    if(bRecording){ //TODO: check filler save correctness
                        save << (int)0 <<", ";
                    }
                }
            }
            
            if(bRecording){
                save << endl;
            }
        }
    }
    
    //now process the newest frame
    int idx = 0; //keep track of the index for continuous buffer reading
    
    frameNum = counter; //set frame num after filler process
    if(bRecording){
        save << frameNum <<", ";
    }
    
    //eeg
    
    //vector<double> fft_chunked; //to CH_NUM * FFT_WINDOW sized vector // TODO: take alternating frames into account when CH_NUM > 8
    
    if(DEF_SERIAL_PACKET_CH_TOTAL  ==  DEF_SERIAL_PACKET_CH_SIZE || counter%2 == 0){
        for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_SIZE; ++i){
            //merge three bytes for 24bit info
            for(unsigned int j = 0; j < 3; ++j, idx++){
                eegByteArray_[j] = buffer[idx];
            }
            
            double filteredValue = scale * interpret24bitAsInt32(eegByteArray_);
            
            if(bRecording){
                save << interpret24bitAsInt32(eegByteArray_) <<", ";
            }
            
            //save raw value
            eeg_raw_v[i].push_back(filteredValue);
            if(eeg_raw_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_raw_v[i].erase(eeg_raw_v[i].begin());
            
            //apply all the filters
            for(int f = 0; f < filter_index.size(); f++){
                if(filter_index[f] >= 0){
                    filteredValue = filters[i][filter_index[f]].filter(filteredValue);
                }
            }
            
            //fft
            fft[i].update(filteredValue);
            VectorDouble fb = fft[i].getFeatureVector();
            for(int j = 0; j < fb.size(); j ++){
                fftBuffer[i][j] = fftBuffer[i][j] * fftSmooth + fb[j] * (1.0 - fftSmooth);
            }
            fft_v[i].push_back(fftBuffer[i]); //TODO: put to other places as well
            if(fft_v[i].size() > SAMPLING_WINDOW) fft_v[i].erase(fft_v[i].begin());
            
            //save processed value
            eeg_v[i].push_back(filteredValue);
            if(eeg_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_v[i].erase(eeg_v[i].begin());
            
            //haar
            if(eeg_v[i].size() >= TRANSFORM_WINDOW){
                haar[i].clear();
                for(int j = 0; j < TRANSFORM_WINDOW; j++){
                    haar[i].push_back(eeg_v[i][eeg_v[i].size() - TRANSFORM_WINDOW + j]);
                }
                //do the transform
                inPlaceFastHaarWaveletTransform_nSweeps(haar[i], TRANSFORM_WINDOW, nSweeps);
            }
        }
        
        // add the new chunked fft vector into the vector of those
        //fft_v.push_back(fft_chunked);
        //if(fft_v.size() > SAMPLING_WINDOW) fft_v.erase(fft_v.begin());
        
        //project through PCA - TODO: add to other places
        if(bSamplingTimeSeriesDone){
            MatrixDouble data(1, DEF_SERIAL_PACKET_CH_TOTAL);
            
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                data[0][i] = eeg_v[i][eeg_v[i].size() - 1];
            }
            
            MatrixDouble prjData;
            if( !pcaTimeSeries.project( data, prjData ) ){
                cout << "ERROR: Failed to project data!\n";
                return EXIT_FAILURE;
            }
            
            for(unsigned int i = 0; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
                eeg_pca[i].push_back(prjData[0][i]);
                if(eeg_pca[i].size() > NUM_MAINTAINED_SAMPLES) eeg_pca[i].erase(eeg_pca[i].begin());
                
                //fft
                fftPCA[i].update(prjData[0][i]);
                VectorDouble fb = fftPCA[i].getFeatureVector();
                for(int j = 0; j < fb.size(); j ++){
                    fftPCABuffer[i][j] = fftPCABuffer[i][j] * fftSmooth + fb[j] * (1.0 - fftSmooth);
                }
            }
        }
    }
    else{
        for(unsigned int i = DEF_SERIAL_PACKET_CH_SIZE; i < DEF_SERIAL_PACKET_CH_TOTAL; ++i){
            //merge three bytes for 24bit info
            for(unsigned int j = 0; j < 3; ++j, idx++){
                eegByteArray_[j] = buffer[idx];
            }
            
            double filteredValue = scale * interpret24bitAsInt32(eegByteArray_);
            
            if(bRecording){
                save << interpret24bitAsInt32(eegByteArray_) <<", ";
            }
            
            //save raw value
            eeg_raw_v[i].push_back(filteredValue);
            if(eeg_raw_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_raw_v[i].erase(eeg_raw_v[i].begin());
            
            //apply all the filters
            for(int f = 0; f < filter_index.size(); f++){
                if(filter_index[f] >= 0){
                    filteredValue = filters[i][filter_index[f]].filter(filteredValue);
                }
            }
            
            //fft
            fft[i].update(filteredValue);
            VectorDouble fb = fft[i].getFeatureVector();
            for(int j = 0; j < fb.size(); j ++){
                fftBuffer[i][j] = fftBuffer[i][j] * fftSmooth + fb[j] * (1.0 - fftSmooth);
            }
            
            //save processed value
            eeg_v[i].push_back(filteredValue);
            if(eeg_v[i].size() > NUM_MAINTAINED_SAMPLES) eeg_v[i].erase(eeg_v[i].begin());
            
            //haar
            if(eeg_v[i].size() >= TRANSFORM_WINDOW){
                haar[i].clear();
                for(int j = 0; j < TRANSFORM_WINDOW; j++){
                    haar[i].push_back(eeg_v[i][eeg_v[i].size() - TRANSFORM_WINDOW + j]);
                }
                //do the transform
                inPlaceFastHaarWaveletTransform_nSweeps(haar[i], TRANSFORM_WINDOW, nSweeps);
            }
        }
    }
    
    //accel
    int tmp_accel[3]; //to check validity of accel data
    for(unsigned int i = 0; i < DEF_SERIAL_PACKET_AXIS_TOTAL; ++i, idx+=2){
        accelByteArray_[0] = buffer[idx];
        accelByteArray_[1] = buffer[idx+1];
        tmp_accel[i] = interpret16bitAsInt32(accelByteArray_);
        
        if(bRecording){
            save << interpret16bitAsInt32(accelByteArray_) <<", ";
        }
    }
    if(tmp_accel[0] == 0 && tmp_accel[1] == 0 && tmp_accel[2] == 0){
        //sampling rate difference mitigation
    }
    else{
        for(unsigned int i = 0; i < DEF_SERIAL_PACKET_AXIS_TOTAL; ++i, idx+=2){
            //save accel data
            accel_v[i].push_back(scale * tmp_accel[i]);
            if(accel_v[i].size() > NUM_MAINTAINED_SAMPLES) accel_v[i].erase(accel_v[i].begin());
        }
    }
    if(bRecording){
        save << endl;
    }
}

//--------------------------------------------------------------
int testApp::interpret16bitAsInt32(unsigned char byteArray[]) {
    int retInt_ = (
                   ((0xFF & byteArray[0]) << 8) |
                   (0xFF & byteArray[1])
                   );
    
    if ((retInt_ & 0x00008000) > 0){
        retInt_ |= 0xFFFF0000;
    }
    else{
        retInt_ &= 0x0000FFFF;
    }
    
    return retInt_;
}

//--------------------------------------------------------------
int testApp::interpret24bitAsInt32(unsigned char byteArray[]){
    int retInt_ = (
                   ((0xFF & byteArray[0]) << 16) |
                   ((0xFF & byteArray[1]) << 8) |
                   (0xFF & byteArray[2])
                   );
    
    if ((retInt_ & 0x00800000) > 0){
        retInt_ |= 0xFF000000;
    }
    else{
        retInt_ &= 0x00FFFFFF;
    }
    return retInt_;
}
