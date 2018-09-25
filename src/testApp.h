
#pragma once

#include "ofMain.h"
#include "GRT.h"
#include "maximilian.h"
#include "FWT.h"

/*
 OpenBCI software for Firmware: v2.0.1, tested on openframeworks v0.9.3
 Note 1: in case your hardware has a different version, the communication protocoal may need adjustment
 Note 2: this software supports both 8- and 16-channel versions, while 16 ch is not comprehensively tested yet
 comments in the code:
    "TODO" - work-in-progress feature that may be removed, or not used yet
    "PARAM" - parameters to be changed depending on needs
    "EXPERIMENTAL" - experimental feature that is not comprehensively tested out
 
 Insturction:
    Addons: ofxFWT and ofxMaxim needed for Haar transform (experimental feature), ofxGRT is required
 
    Hardware hook-up:
        1) plug in the OpenBCI USB dongle
        2) properly wear and turn on the OpenBCI headgear
        3) run the software.
 
    Software initiation:
        1) if successful you will see "return" on the top right corner
        2) if you don't see "return"
            - try turning off/on the headgear
            - if this doesn't work, exit the software, and plug in the dongle again + reboot the head gear and try from beginning
 
    Getting the signal
        1) once "ready" is dislayed, hit "space" key and data transmission will start
        2) check how much noise happens from your environment [important]
            if too much noise, below tips may be helpful
                - the reference electrodes are critical. try making it as firmly attached to your ear as possible
                - removing shoes is often helpful in reducing electrical nosie
                - ideally, body movement should not affect the EEG signals. if noise happens, try adjusting your environment such as body posture in order to make it as clean as possible
                - restarting the software and the headgear can be helpful sometimes
                - getting in touch with electronic devices may add 50-60hz noise. in an ideal setting, this will not happen.
 
    Data processing
        1) once the transmission begins, you will see EEG signals displayed
        2) hit 't' key to perform PCA on the input signals. this will reduce the dimension of signal for data processing
        3) hit 's' key to sample training data
            - there will be delay between the key input and the sample recording, in order for an experimenter to have time to get prepared
            - after recording, training label will automaticall increase its index. use '<' and '>' buttons to change the training label
            - once all the samples are collected, hit 'd' key to traing SVM
            - after training is done, the screen will show classification results in real time
 
    Other key shortcuts
        left and right arrow keys - use that to cycle through different visualizations
            - we have time-series data, fft, haar transform, time-series data after PCA, fft after PCA, and classification data
        'n' key - enable/disable notch filter (removing noise from power lines)
        'b' key - cycle through different bandpass filter (default one is usable)
        'i' key - reset training data
        * check void keyPressed(int key) for other key shortcuts
 
    Filter definition
        define filter coefficients in setup() function. use MATLAB functions to create IIR filter
        after defining coefficients, add to the filter vector (can be found in the setup() function)
 
    Other
        read through the code to see other parameter settings
        contact sangwon@media.mit.edu for additional questions
 
*/


using namespace GRT;

#define DEF_SERIAL_PORT              "/dev/tty.usbserial-DQ0084BE" //PARAM - set it to your OpenBCI USB dongle
#define DEF_SERIAL_BAUDRATE          (115200)
#define DEF_SERIAL_PACKET_EOP        (0xC0)
#define DEF_SERIAL_PACKET_PREFIX     (0xA0)
#define DEF_SENSOR_STREAMING_START   ('b')
#define DEF_SENSOR_STREAMING_STOP    ('s')
#define DEF_SENSOR_RESET             ('v')
#define STATE_EOP                      0
//#define STATE_READY                  1 //not used
#define STATE_READING                  2

#define DEF_SERIAL_PACKET_AXIS_TOTAL (3)
#define DEF_SERIAL_PACKET_CH_TOTAL   (8) //PARAM - set it to 16 if you use 16 channels
#define DEF_SERIAL_PACKET_CH_SIZE   (8)

#define FILTER_MAX_WINDOW 5 //PARAM - maximum size for self-defined IRR filters
#define NUM_VIEW_MODE 8 //PARAM - change this number if you add/remove viewmodes
#define TRANSFORM_WINDOW 64 //PARAM - window for FFT and Haar transformations
#define SAMPLING_WINDOW 128 //PARAM - sampling window for PCA and so on
#define num_bands 5 //PARAM - number of bands for bandpower calculation. not critical

#define NUM_MAX_CLASSES 3 //PARAM - IMPORTANT: number of classes to be used for classification
#define LABELING_WINDOW 40 //PARAM - sampling window for smoothing classification

//PARAMS - all below definitions are for visualizations
#define NUM_MAINTAINED_SAMPLES 500
#define PLOT_SCALE_Y 0.5
#define PLOT_SCALE_X 2
#define PLOT_SCALE_X_FFT 4
#define PLOT_SCALE_Y_LOG 500
#define PLOT_SCALE_Y_LIKELIHOOD 200


class IIR //TODO: also make FIR filter class
{
    string name;
    
    vector<double> a, b, x, y; // a[1:an] b[0:bn] y[1:yn] x[0:xn] // a: coeff for y
    int max_filter_size;
    
    
public:
    IIR(int _size, string _name = "untitled"){
        max_filter_size = _size;
        name = _name;
    }
    
    void setCoefficients(vector<double> _a, vector<double> _b){
        a = _a;
        b = _b;
    }
    
    string getName(){
        return name;
    }
    
    double filter(double x0)
    {
        x.push_back(x0); //add the new sample into the x frames
        if(x.size() > max_filter_size) x.erase(x.begin());
        
        int na = a.size();
        int nb = b.size();
        int nx = x.size();
        int ny = y.size();
        
        double ret = 0;
        
        for(int i = 0; i < nb; i ++){
            if(nx - 1 - i >= 0){
                ret += b[i] * x[nx - 1 - i];
            }
        }
        for(int i = 0; i < na; i ++){
            if(ny - 1 - i >= 0){
                ret -= a[i] * y[ny - 1 - i];
            }
        }
        
        y.push_back(ret);
        if(y.size() > max_filter_size) y.erase(y.begin());
        
        return ret;
    }
};


class testApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    int  interpret24bitAsInt32(unsigned char byteArray[]);
    int  interpret16bitAsInt32(unsigned char byteArray[]);
    void processFrames();
    
    //aux
    int view_mode = 0; // 0: 1-8, 1: 9-16, 2: fft
    
    //transmission
    ofSerial  _serial;
    bool enabled = false;
    bool startedStreaming = false;
    bool isReady = false;
    int initSequenceCount;
    vector<unsigned char> buffer, prev_buffer; //prev buffer used only for debuggin
    int frameNum, readState;
    int numRequiredBytes;
    long long t_sample;
    
    //data received
    unsigned char read         = 0;
    unsigned char msb          = 0;
    unsigned char lsb          = 0;
    unsigned char counter      = 0;
    unsigned char eegByteArray_[3];
    unsigned char accelByteArray_[2];
    unsigned char flagByte;
    
    //raw and fully processed data: over frames
    vector<double> eeg_v[DEF_SERIAL_PACKET_CH_TOTAL];
    vector<double> eeg_raw_v[DEF_SERIAL_PACKET_CH_TOTAL];
    vector<double> accel_v[DEF_SERIAL_PACKET_AXIS_TOTAL];
    vector<double> eeg_pca[DEF_SERIAL_PACKET_CH_TOTAL];
    double scale;
    
    //filters
    int freq;
    vector<int> filter_index;
    vector<IIR> filters[DEF_SERIAL_PACKET_CH_TOTAL];
    
    //FFT
    FFT fft[DEF_SERIAL_PACKET_CH_TOTAL];
    bool fft_active[DEF_SERIAL_PACKET_CH_TOTAL];
    double minlog, maxlog;
    vector<double> fftBuffer[DEF_SERIAL_PACKET_CH_TOTAL];
    FFT fftPCA[DEF_SERIAL_PACKET_CH_TOTAL];
    vector<double> fftPCABuffer[DEF_SERIAL_PACKET_CH_TOTAL];
    vector<vector<double>> fft_v[DEF_SERIAL_PACKET_CH_TOTAL]; //collect all channels into single rows
    double fftSmooth;
    
    //band power
    bool bShowBandpower = false;
    int processing_band_low_Hz[num_bands] = {
        1, 4, 8, 13, 30
    }; //lower bound for each frequency band of interest (2D classifier only)
    int processing_band_high_Hz[num_bands] = {
        4, 8, 13, 30, 55
    };  //upper bound for each frequency band of interest
    
    double bandPower[num_bands] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    //haar wavelet
    int nSweeps;
    vector<double> haar[DEF_SERIAL_PACKET_CH_TOTAL];
    
    //analysis
    PrincipalComponentAnalysis pcaTimeSeries;
    PrincipalComponentAnalysis pcaFFT;
    FFTFeatures fftFeatures[DEF_SERIAL_PACKET_CH_TOTAL]; //TODO: not used
    
    //sampling learning data
    bool bSamplingTimeSeries = false;
    bool bSamplingFFT = false;
    bool bSamplingTimeSeriesDone = false;
    bool bSamplingFFTDone = false;
    
    //read data from file
    bool bReadFromFile = false; //PARAM - change it to true if reading from a file. file format is specific to this software.
    ofFile file;
    ofBuffer readBuffer;
    vector<string> readData;
    long long t_sample_read;
    bool signDirect = true; // PARAM: if sign of a value need to be translated directly when reading from txt file
    
    //save to file
    ofFile save;
    bool bRecording = false;
    
    //classification
    int trainingLabel, classifiedLabel;
    long long t_sampling_duration = 6000;
    long long t_sampling_start;
    long long t_sampling_delay = 2000;
    SVM svm_ts, svm_fft;
    bool bSampleRecording = false;
    bool bTrained = false;
    GRT::ClassificationData trainingData; //This will store our training data
    
    //processing classificatiion results
    vector<float> lh[NUM_MAX_CLASSES];
    long long t_class_likelihood;
    vector<float> lh_ratio[NUM_MAX_CLASSES];
    vector<int> labelRaw;
    vector<int> labelFiltered;
    bool bPrintLikelihood;
};
