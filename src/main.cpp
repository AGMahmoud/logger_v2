/* WaveShare IMU
 * u-blox-8 GPS PVT(GNGGA) and RAWX
 */
 
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <chrono>
#include <sstream>
#include <typeinfo>
#include <math.h>
#include <sys/ioctl.h>
#include <mutex>
#include <thread>
#include <vector>		//std::vector
#include <numeric>      //std::accumulate
#include <condition_variable>
#include <ctime>
#include "./ems_data_structures.h"
#include "serial.hpp"
#include "util.h"
#include "Zed_cpu_2.hpp"





using namespace std;

int imu_freq = 100;
int imu_period = 1000000000/imu_freq;

#define IMU_PERIOD	imu_period	// In nano second
#define G_VALUE 9.8				// Gravity constant
#define I2C_IMU_ADDRESS 0x68	// MPU9250 accel and gyro address in 7 bits with 1 bit zero padding at msb
#define I2C_MAG_ADDRESS	0x0C	// MPU9250 mag address in 7 bits with 1 bit zero padding at msb
#define I2C_BARO_ADDRESS 0x77	// BMP280 baro address in 7 bits with 1 bit zero padding at msb

void *ImuWrite(void*);
void *ImuWrite_2(void*);
void *GpsWrite(void*);
int InitTimer(long long, timer_t , void (*handler)(int, siginfo_t*, void*));
int InitTimer_2(long long, timer_t , void (*handler)(int, siginfo_t*, void*));
void ImuTimerExpiration(int, siginfo_t*, void*);
void ImuTimerExpiration_2(int, siginfo_t*, void*);
void Imu_Initialization();
void Imu_Initialization_2();
double CompensatePressure(int , int);
int hex2dec(int, int, char *);
void decode_gps_data(char *);
void OpenGpio(int, int, int&);
void CloseGpio(int&);
void OpenSerialPort(const char*, int&, int, int);
void CloseSerialPort(int&);
void OpenI2CPort(const char*, int&);
void CloseI2CPort(int&);
void I2CWriteBytes(int, char, char*, char);
void I2CWriteBytes_2(int, char, char*, char);
void I2CReadBytes(int, char, char, char*, char);
void I2CReadBytes_2(int, char, char, char*, char);
void open_new_file();
void Stop(int);
char *sstrstr(char *, char *, size_t );
double ubx_r(int, int, char *);
void Yei_init(uint32_t update_Rate, uint32_t streamDuration, uint32_t initDelay, int IMU_no);
uint8_t Yei_checksum(std::vector<uint8_t> data);
void* Yei_update();
std::mutex moduleIoMutex;
//~ std::mutex mcpw_sync_response_mutex;
//~ std::condition_variable mcpw_sync_response_condition;

timer_t imu_timer;
//timer_t imu_timer_2;
timer_t gps_timer;
static int timer_count   = 0;			// Number of timers instantiated in this code
static int timer_count_1 = 0;

int imu_fd = -1;
int imu_fd_2 = -1;
int gps_fd = -1;

//const char *gps_num   = "/dev/ttyTHS1";	// On J41 header.
//const char *gps_num   = "/dev/ttyTHS1";	// On J41 header.
const char *gps_num   = "/dev/ttyACM1";	// On J41 header.
std::string YOST_port="/dev/ttyACM0";
Serial *serial,*serial2;
//const char *imu_num   = "/dev/i2c-1";		// on J41 header
//const char *imu_num_2 = "/dev/i2c-0";		// on J41 header

double imu_time_tag;
//double imu_time_tag_2;
double gps_time_tag;
double gps_prgrm_time_tag;

double accuracy;

pthread_cond_t imu_data_ready_cond;
pthread_mutex_t imu_data_ready_mutex;

//pthread_cond_t imu_data_ready_cond_2;
//pthread_mutex_t imu_data_ready_mutex_2;


pthread_mutex_t file_write_mutex;

//pthread_mutex_t gps_file_write_mutex;

char all_file_path[100];
char file_counter = 1;
//char gps_file_path[100];
//char gps_file_counter = 1;

bool imu_run   = true;
//bool imu_run_2 = true;
bool gps_run   = true;

auto time_start = std::chrono::system_clock::now();
auto imu_time_now = std::chrono::system_clock::now();
auto ws_imu_time_now = std::chrono::system_clock::now();
//auto imu_time_now_2 = std::chrono::system_clock::now();
//auto ws_imu_time_now_2 = std::chrono::system_clock::now();
auto gps_prgrm_time_now = std::chrono::system_clock::now();
auto test_start = std::chrono::system_clock::now();
auto test_end = std::chrono::system_clock::now();

unsigned int imu_packet_counter     = 0;
//unsigned int imu_packet_counter_2   = 0;
unsigned int gps_pvt_packet_counter = 0;
unsigned int gps_raw_packet_counter = 0;

unsigned int conv_hex2dec_x;
stringstream ss;
int conv_hex2dec, offset_data, numSV;
unsigned char Pvt[4]={0xB5,0x62,0x01,0x07};
unsigned char Raw[4]={0xB5,0x62,0x02,0x15};
//~ char CFG_MSG_PVT[11]={0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x07,0x01,0x13,0x51};
double doMes, mesQI, rcvTow, sv ;
double range_rate_const = 299792458.0/1575420000.0;
union { uint64_t ubx_i; float ubx_f; double ubx_d;} ubx_r_conv;
Double64 sat_id[40]        = {};
double range[40]      = {};
double range_rate[40] = {};

int temp_comp_param[3];			// BMP280 temperature compensation parameters
int press_comp_param[9];		// BMP280 pressure compensation parameters
double mag_sens[3]; 			// MPU9250 magnetometer scale adjustment parameters

int temp_comp_param_2[3];			// BMP280 temperature compensation parameters
int press_comp_param_2[9];		// BMP280 pressure compensation parameters
double mag_sens_2[3]; 			// MPU9250 magnetometer scale adjustment parameters

FILE *all_file;
//FILE *gps_file;

int main() {
    cout << "\n------------- Welcome to IMU, GPS, RGBD, UWB Logger on Jetson project -------------" << endl;
    cout << "\nPress cntl+z to stop" << endl;

    signal(SIGTSTP, Stop);

    open_new_file();

    //OpenI2CPort(imu_num, imu_fd);
    //OpenI2CPort(imu_num_2, imu_fd_2);
    OpenSerialPort(gps_num, gps_fd, 100, 5);
    serial = new Serial(YOST_port,115200);
    //serial2 = new Serial(gps_num,115200);
   // Imu_Initialization();
   // Imu_Initialization_2();
    Yei_init(0x00002710,0xFFFFFFFF,0x00000000,1);
    pthread_t imu_thread;
    //pthread_t imu_thread_2;
    pthread_t gps_thread;
    pthread_t zed_thread;

    pthread_attr_t attr;
    void *status;

    long imu_thread_id = 0;
    //long imu_thread_id_2 = 3;
    long gps_thread_id = 2;
    long zed_thread_id = 4;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    pthread_mutex_init(&file_write_mutex, NULL);
    //pthread_mutex_init(&gps_file_write_mutex, NULL);

    pthread_mutex_init(&imu_data_ready_mutex, NULL);
    pthread_cond_init(&imu_data_ready_cond, NULL);

    //~ pthread_mutex_init(&imu_data_ready_mutex_2, NULL);
    //~ pthread_cond_init(&imu_data_ready_cond_2, NULL);

    pthread_create(&gps_thread, &attr, GpsWrite, (void *)gps_thread_id);
    pthread_create(&imu_thread, &attr, ImuWrite, (void *)imu_thread_id);
    //pthread_create(&imu_thread_2, &attr, ImuWrite_2, (void *)imu_thread_id_2);
    pthread_create(&zed_thread, &attr, zed_main, (void *)zed_thread_id);
    pthread_attr_destroy(&attr);

    pthread_join(imu_thread, &status);
    cout << "\nIMU thread Joined";

    //pthread_join(imu_thread_2, &status);
   // cout << "\nIMU thread 2 Joined";

    pthread_join(gps_thread, &status);
    cout << "\nGPS Thread Joined";
    pthread_join(zed_thread, &status);
    cout << "\nZED thread Joined";

    fclose(all_file);
    cout << "\nfile close";

   // fclose(gps_file);
   // cout << "\nGPS file close";

    CloseSerialPort(gps_fd);
    //CloseI2CPort(imu_fd);
    //CloseI2CPort(imu_fd_2);

    return 0;
}


void *GpsWrite(void *thread_id){

    Uint8 sat_id[32]= {0};
    Double64 range[32]= {0};
    Double64 range_rate[32]={0};

    struct EmsIcdGpsPVTStruct gps_measurement_pvt;
    gps_measurement_pvt.packet_id    = (Uint8) GPSPVT_PACKET_ID;
    gps_measurement_pvt.status 		 = (Uint8) 0x01;
    gps_measurement_pvt.latitude_std = (Double64) 0.0;
    gps_measurement_pvt.longitude_std= (Double64) 0.0;
    gps_measurement_pvt.altitude_std = (Double64) 0.0;
    gps_measurement_pvt.vn_std		 = (Double64) 0.0;
    gps_measurement_pvt.ve_std 		 = (Double64) 0.0;
    gps_measurement_pvt.vd_std		 = (Double64) 0.0;
    gps_measurement_pvt.time_tag     = (Double64) 0.0;
    gps_measurement_pvt.gps_time_tag = (Double64) 0.0;
    gps_measurement_pvt.latitude     = (Double64) 0.0;
    gps_measurement_pvt.longitude    = (Double64) 0.0;
    gps_measurement_pvt.altitude     = (Double64) 0.0;
    gps_measurement_pvt.accuracy	 = (Double64) 0.0;
    gps_measurement_pvt.speed        = (Double64) 0.0;
    gps_measurement_pvt.bearing      = (Double64) 0.0;
    gps_measurement_pvt.vn           = (Double64) 0.0;
    gps_measurement_pvt.ve           = (Double64) 0.0;
    gps_measurement_pvt.vd           = (Double64) 0.0;

    struct EmsRawGNSSStruct gps_measurement_raw;
    gps_measurement_raw.packet_id    = (Uint8) GPSRAW_PACKET_ID;
    gps_measurement_raw.status 		 = (Uint8) 0x01;
    gps_measurement_raw.no_svs 		 = (Uint8) 0x00;
    gps_measurement_raw.time_tag     = (Double64) 0.0;
    gps_measurement_raw.gps_time_tag = (Double64) 0.0;

    int b_size = 1000;
    char read_buffer[b_size];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */

    //tcflush(gps_fd, TCIFLUSH);
    double old_time_tag = 0; double current_time_tag = 1;
    double old_time_tag_r = 0; double current_time_tag_r = 1;

    while(gps_run){
        //tcflush(gps_fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
        bytes_read = read(gps_fd, &read_buffer, sizeof(read_buffer)); /* Read the data                   */
	//cout<<read_buffer<<endl;
	//serial2->get((uint8_t*)read_buffer, 1000);
	//cout<<(char*)read_buffer<<endl;

        gps_prgrm_time_now = std::chrono::system_clock::now();
        gps_prgrm_time_tag = chrono::duration<double>(gps_prgrm_time_now-time_start).count();

        read_buffer[bytes_read] = 0; //terminate the string

        char *data_pvt = sstrstr(read_buffer, (char*)Pvt, sizeof(read_buffer));
        char *data_raw = sstrstr(read_buffer, (char*)Raw, sizeof(read_buffer));

      /*   for(int i = 0; i<80;i++){
             if(i%16 == 0){ cout << endl;}
             printf("%x ",data_pvt[i]);
         }*/
       if ( data_pvt != NULL ) {

            int GPS_fix_type = hex2dec(1, 26, data_pvt);
            current_time_tag = (Double64) hex2dec(4, 6, data_pvt)  * pow(10, -3);
            if (GPS_fix_type != 0x00 && (old_time_tag != current_time_tag)) {
                //~ cout << "\n*GPS_PVT"<<endl;

                gps_measurement_pvt.status = (Uint8) 0x00;
                gps_measurement_pvt.time_tag     = (Double64) gps_prgrm_time_tag;
                gps_measurement_pvt.gps_time_tag = (Double64) hex2dec(4, 6, data_pvt)  * pow(10, -3);
                gps_measurement_pvt.latitude     = (Double64) hex2dec(4, 34, data_pvt) * pow(10,-7);
                gps_measurement_pvt.longitude    = (Double64) hex2dec(4, 30, data_pvt) * pow(10,-7);
                gps_measurement_pvt.altitude     = (Double64) hex2dec(4, 38, data_pvt) * pow(10, -3); //height above Ellipsoid
                gps_measurement_pvt.accuracy	 = (Double64) sqrt( pow(hex2dec(4, 46, data_pvt),2)+ pow(hex2dec(4, 50, data_pvt), 2)) * pow(10, -3);
                gps_measurement_pvt.speed        = (Double64) hex2dec(4, 66, data_pvt) * pow(10, -3);
                gps_measurement_pvt.bearing      = (Double64) hex2dec(4, 70, data_pvt) * pow(10, -5);
                gps_measurement_pvt.vn           = (Double64) hex2dec(4, 54, data_pvt) * pow(10, -3);
                gps_measurement_pvt.ve           = (Double64) hex2dec(4, 58, data_pvt) * pow(10, -3);
                gps_measurement_pvt.vd           = (Double64) hex2dec(4, 62, data_pvt) * pow(10, -3);


                std::cout << "\nGPS PVT is logging. Current packet ID: " << gps_pvt_packet_counter<<endl;
                pthread_mutex_lock(&file_write_mutex);
                    fwrite(&gps_measurement_pvt, 1, sizeof(gps_measurement_pvt), all_file);
                pthread_mutex_unlock(&file_write_mutex);
                //~ printf("\ntime_tag: %f\n",gps_measurement_pvt.time_tag);
                //~ printf("gps_time_tag: %f\n",gps_measurement_pvt.gps_time_tag);
                //~ printf("Lat: %f\n",gps_measurement_pvt.latitude);
                //~ printf("Long: %f\n",gps_measurement_pvt.longitude);
                if(gps_pvt_packet_counter == 0){
                    current_time_tag = gps_measurement_pvt.gps_time_tag;
                    old_time_tag = current_time_tag;
                }else{
                    old_time_tag = gps_measurement_pvt.gps_time_tag;
                }

                gps_pvt_packet_counter++;
            }
        }

        if (data_raw != NULL) {

            current_time_tag = (Double64) (Double64) rcvTow;

            if(old_time_tag != current_time_tag){
                rcvTow  = (Double64) ubx_r(8, 6, data_raw);
                numSV 	= hex2dec(1, 6+11, data_raw);
                //~ cout << "\n*numSV: " << numSV << "\trcvTow:"<<rcvTow<< endl;

                gps_measurement_raw.status 		 = (Uint8) 0x00;
                gps_measurement_raw.no_svs 		 = (Uint8) numSV;
                gps_measurement_raw.time_tag     = (Double64) gps_prgrm_time_tag;
                gps_measurement_raw.gps_time_tag = (Double64) rcvTow;

                //~ if (numSV > 0 && numSV <40 && rcvTow > 0){
                if (numSV > 0){
                    for (int i=0; i<numSV; i++){

                        //mesQI = hex2dec(1, 6+29+24*i, data_raw);  // Not present in ublox-8

                        //~ if(mesQI > 3  && mesQI <15){

                            sat_id[i]		= (Uint8) hex2dec(1, 6+37+32*i, data_raw);
                            range[i]		= (Double64) ubx_r(8, 6+16+32*i, data_raw);
                            doMes			= ubx_r(4, 6+32+32*i, data_raw);
                            range_rate[i]	= (Double64) doMes*range_rate_const;
                            //~ range_rate[i]	= (Double64) doMes;	// just for debugging

                        //~ }
                    }

                    pthread_mutex_lock(&file_write_mutex);
                        fwrite(&gps_measurement_raw, 1, sizeof(gps_measurement_raw), all_file);	//writing the fixed part of the RAWGNSS structure
                        fwrite(sat_id, 1, sizeof(Uint8)*numSV, all_file);	//writing the varibale length part of the RAWGNSS structure
                        fwrite(range, 1, sizeof(Double64)*numSV, all_file);	//writing the varibale length part of the RAWGNSS structure
                        fwrite(range_rate, 1, sizeof(Double64)*numSV, all_file);	//writing the varibale length part of the RAWGNSS structure
                    pthread_mutex_unlock(&file_write_mutex);

                    if(gps_raw_packet_counter == 0){
                        current_time_tag = gps_measurement_raw.gps_time_tag;
                        old_time_tag = current_time_tag;
                    }else{
                        old_time_tag = gps_measurement_raw.gps_time_tag;
                    }

                    gps_raw_packet_counter++;

                 }


                //~ if (numSV>0){		//  just for debug
                    //~ cout << "\nNew epoch: Time. Raw time tag: " << gps_measurement_raw.gps_time_tag << " PVT time Tag:" << gps_measurement_raw.gps_time_tag;
                    //~ for (int i = 0; i<numSV; i++){
                        //~ printf ("\nSatID: %d\t Range: %f\t Range Rate: %f", sat_id[i], range[i], range_rate[i]);
                    //~ }
                    //~ cout << "\n";
                    //~ cout << "\n";
                //~ }
                //~ cout<<"\n*Check: "<<sizeof(sat_id)<<"\t"<<sizeof(range)<<"\t"<<sizeof(range_rate)<<endl;
            }



        }
        cout << "\r\t\t\b\b\b,PVT: " << gps_pvt_packet_counter << "\r\t\t\t,RAW: " << gps_raw_packet_counter;
    }
    cout << "\nGPS Thread Exited";
    pthread_exit(NULL);

}

void *ImuWrite(void *thread_id){
    int buffer_size = 100;
    int buffer_index = 0;

    struct EmsIcdImuStruct imu_measurement[buffer_size];
    struct EmsIcdMagStruct mag_measurement[buffer_size];
    //struct EmsIcdBaroStruct baro_measurement[buffer_size];

    for (int i=0; i<buffer_size; i++){
        imu_measurement[i].packet_id = (Uint8) 0x01;
        imu_measurement[i].status = (Uint8) 0x00;

       /* baro_measurement[i].packet_id = (Uint8) 0x03;
        baro_measurement[i].status = (Uint8) 0x00;
        baro_measurement[i].baro_stdv = (Double64) 0x00;*/

        mag_measurement[i].status= (Uint8) 0x00;
        mag_measurement[i].packet_id = (Uint8) 0x04;
        mag_measurement[i].roll = (Double64) 0x00;
        mag_measurement[i].pitch = (Double64) 0x00;
        mag_measurement[i].heading = (Double64) 0x00;
        mag_measurement[i].heading_std = (Double64) 0x00;
    }

    Double64 accel_raw[3];
    Double64 gyro_raw[3];
    Float32 mag_raw[3];
    Double64 euler_raw[3];
    InitTimer((long long) IMU_PERIOD, imu_timer, &ImuTimerExpiration);
    cout << "IMU 1 Thread" <<endl;
    while (imu_run){
        //~ cout << "\nIN while IMU-1"<<endl;
        pthread_mutex_lock(&imu_data_ready_mutex);
            pthread_cond_wait(&imu_data_ready_cond, &imu_data_ready_mutex);
        pthread_mutex_unlock(&imu_data_ready_mutex);

        //~ cout << "IMU_1 Next!" <<endl;

        /*I2CReadBytes(imu_fd, I2C_IMU_ADDRESS, 0x3B, accel_buff, 6);	//read accelerometer data
        I2CReadBytes(imu_fd, I2C_IMU_ADDRESS, 0x43, gyro_buff, 6);	//read gyrso data
        I2CReadBytes(imu_fd, I2C_BARO_ADDRESS, 0xF7, baro_buff, 6); //read barometer and temperature data
        I2CReadBytes(imu_fd, I2C_MAG_ADDRESS, 0x02, mag_buff, 8);	//read magnetometer data*/
	uint8_t rawData[53];
	std::vector<float> inData;
	inData.reserve(6);
		
	int bytesReceived = serial->get(rawData, 53);
	if(bytesReceived == 53){
	//*ignore the random unwanted first byte*
	uint32_t inTimeStamp = bytesToInt(&rawData[1]); //<- timestamp from device; 
	//fprintf(IMU_Data,"%d\t",inTimeStamp);
	//std::cout<<"Timestamp\t"<<inTimeStamp<<std::endl;
	//uint32_t inTimeStamp = GetTickCount(); 		//<- use the RPi clock for timestamps
	for(int i = 5; i < bytesReceived; i = i + 4){
	//format into floating point and append to data vector
	appendFloat(&rawData[i],inData);
	}
	accel_raw[0]=inData.at(0);
	accel_raw[1]=inData.at(1);
	accel_raw[2]=inData.at(2);
	gyro_raw[0] =inData.at(3);
	gyro_raw[1] =inData.at(4);
	gyro_raw[2] =inData.at(5);
	mag_raw[0]  =inData.at(6);
	mag_raw[1]  =inData.at(7);
	mag_raw[2]  =inData.at(8);
	euler_raw[0]=inData.at(9);
	euler_raw[1]=inData.at(10);
	euler_raw[2]=inData.at(11);
	mag_measurement[buffer_index].status= (Uint8) 0x00;

	imu_time_tag = chrono::duration<double>(imu_time_now-time_start).count();

        imu_measurement[buffer_index].time_tag = imu_time_tag;
        mag_measurement[buffer_index].time_tag = imu_time_tag;

 //	Changing the scale of accelerometer and gyroscope these scales should change
        imu_measurement[buffer_index].acc_x = accel_raw[0] ;	// in m/s^2
        imu_measurement[buffer_index].acc_y = accel_raw[1];	// in m/s^2
        imu_measurement[buffer_index].acc_z = accel_raw[2] ;	// in m/s^2
        imu_measurement[buffer_index].gyro_x = gyro_raw[0]/65.5;	// in deg/s
        imu_measurement[buffer_index].gyro_y = gyro_raw[1]/65.5;	// in deg/s
        imu_measurement[buffer_index].gyro_z = gyro_raw[2]/65.5;	// in deg/s

	mag_measurement[buffer_index].mag_x = mag_raw[0];	// in uT
        mag_measurement[buffer_index].mag_y = mag_raw[1];	// in uT
        mag_measurement[buffer_index].mag_z = mag_raw[2];	// in uT
	mag_measurement[buffer_index].roll = euler_raw[0];	// in rad
        mag_measurement[buffer_index].pitch = euler_raw[1];	// in rad
        mag_measurement[buffer_index].heading = euler_raw[2];



	//std::cout<<imu_time_tag<<"\t"<<accel_raw[0]<<"\t"<<accel_raw[1]<<"\t"<<accel_raw[2]<<"\t"<<gyro_raw[0]/65.5<<"\t"<<gyro_raw[1]/65.5<<"\t"<<gyro_raw[2]/65.5<<endl;
        

        if (pthread_mutex_trylock(&file_write_mutex) == 0){
            //~ cout << "\nIn try IMU_1";
            fwrite(&imu_measurement, 1, (buffer_index+1)*sizeof(EmsIcdImuStruct), all_file);
            fwrite(&mag_measurement, 1, (buffer_index+1)*sizeof(EmsIcdMagStruct), all_file);
           // fwrite(&baro_measurement, 1, (buffer_index+1)*sizeof(EmsIcdBaroStruct), all_file);
            pthread_mutex_unlock(&file_write_mutex);
            buffer_index = 0;

        } else if (buffer_index == (buffer_size-1)){
            //~ cout << "\nIn lock";
            pthread_mutex_lock(&file_write_mutex);
                fwrite(&imu_measurement, 1, (buffer_index+1)*sizeof(EmsIcdImuStruct), all_file);
                fwrite(&mag_measurement, 1, (buffer_index+1)*sizeof(EmsIcdMagStruct), all_file);
              //  fwrite(&baro_measurement, 1, (buffer_index+1)*sizeof(EmsIcdBaroStruct), all_file);
            pthread_mutex_unlock(&file_write_mutex);
            buffer_index = 0;

        } else{
             buffer_index += 1;
            //~ cout << "\n increment: " << buffer_index;
        }

        imu_packet_counter++;

        if (imu_packet_counter%imu_freq ==0)
            cout << "\nIMU_1: " <<  imu_packet_counter;
	
	}		
			

       
        
       

    }

    cout << "\nIMU_1 Thread Exited";
    pthread_exit(NULL);
}

char *sstrstr(char *haystack, char *needle, size_t length){
    size_t needle_length = strlen(needle);
    size_t i;

    for (i = 0; i < length; i++)
    {
        if (i + needle_length > length)
        {
            return NULL;
        }

        if (strncmp(&haystack[i], needle, needle_length) == 0)
        {
            return &haystack[i];
        }
    }
    return NULL;
}

int hex2dec(int a_size, int offset, char *a){

    if (a_size == 4){
        conv_hex2dec = int((( *(a+3+offset) <<8)| *(a+2+offset) ) << 16) | (( *(a+1+offset) <<8)| *(a+offset) );
    }
    else if (a_size == 2){
        conv_hex2dec = int(a[offset+1]<<8)|a[offset];
    }
    else if (a_size == 1){
        conv_hex2dec = int(a[offset]);
    }

    ss << hex << conv_hex2dec;
    ss >> conv_hex2dec_x;
    ss.clear();

    return static_cast<int>(conv_hex2dec_x);
}

double ubx_r(int a_size, int offset, char *a){

    if (a_size == 4){
        ubx_r_conv.ubx_i = ((( *(a+3+offset) <<8)| *(a+2+offset) ) << 16) | (( *(a+1+offset) <<8)| *(a+offset) );
        return ubx_r_conv.ubx_f;
    }
    else if (a_size == 8){
        ubx_r_conv.ubx_i  = (((( *(a+7+offset) <<8)| *(a+6+offset) ) << 16) | (( *(a+5+offset) <<8)| *(a+4+offset) ));
        ubx_r_conv.ubx_i  = (ubx_r_conv.ubx_i  << 32) | uint32_t(((( *(a+3+offset) <<8)| *(a+2+offset) ) << 16) | (( *(a+1+offset) <<8)| *(a+offset) ));
        return ubx_r_conv.ubx_d;
    }

}

double CompensatePressure(int adc_P, int adc_T){
    double var1, var2, pressure;
    double t_fine;

    //adc_P = 415148;
    //adc_T = 519888;
    // These equations obtaiend from datasheet
    var1 = (((double) adc_T) / 16384.0 - ((double) temp_comp_param[0]) / 1024.0) * ((double) temp_comp_param[1]);
    var2 = ((((double) adc_T) / 131072.0 - ((double) temp_comp_param[0]) / 8192.0)  * (((double) adc_T) / 131072.0 - ((double) temp_comp_param[0]) / 8192.0)) * ((double) temp_comp_param[2]);
    t_fine = (var1 + var2);
    //double temperature = (var1 + var2) / 5120.0;
    //printf ("\nTemp: %f", temperature);

    var1 = ((double) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) press_comp_param[5]) / 32768.0;

    var2 = var2 + var1 * ((double) press_comp_param[4]) * 2.0;
    var2 = (var2 / 4.0) + (((double) press_comp_param[3]) * 65536.0);
    var1 = (((double) press_comp_param[2]) * var1 * var1 / 524288.0  + ((double) press_comp_param[1]) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) press_comp_param[0]);

    if (var1 == 0.0) {
        return 0; // avoid exception caused by division by zero
    }

    pressure = 1048576.0 - (double) adc_P;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) press_comp_param[8]) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double) press_comp_param[7]) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double) press_comp_param[6])) / 16.0;

    return pressure;
}

void ImuTimerExpiration(int sig, siginfo_t *si, void *uc){

    imu_time_now = std::chrono::system_clock::now();

    pthread_mutex_lock(&imu_data_ready_mutex);
        pthread_cond_signal(&imu_data_ready_cond);
    pthread_mutex_unlock(&imu_data_ready_mutex);

    }



void Imu_Initialization(){
    char wrt_buff[4] = {0,0,0,0};	// [0] should be register address [1:] should be the value(s) to be writen

    // initialization of MPU9250 Accelerometer and Gyroscope
    wrt_buff[0] = 0x6B;
    wrt_buff[1] = 0x00;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1); //power management (run mode)

    wrt_buff[0] = 0x19;
    wrt_buff[1] = 0x09;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1); //Sample rate 1000/(1+9) = 100Hz

    wrt_buff[0] = 0x1A;
    wrt_buff[1] = 0x00;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1); //gyro and temperature Low Pass filter 250Hz 0.97ms delay

    wrt_buff[0] = 0x1B;
    wrt_buff[1] = 0x08;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1); //set to 500dps. Gyro Range: [4:3]= 00:250dps, 01:500dps, 10:1000dps, 11:2000dps. If changed, scaling in the IMU_write should change accordingly

    wrt_buff[0] = 0x1C;
    wrt_buff[1] = 0x08;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1); //set to 4g. Accel Range: [4:3]= 00:2g, 01:4g, 10:8g, 11:16g. If changed, scaling in the IMU_write should change accordingly

    wrt_buff[0] = 0x1D;
    wrt_buff[1] = 0x00;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1); //accel Low Pass filter 218Hz 1.88ms delay

    // initilization of MPU9250 Magnetometer
    wrt_buff[0] = 0x6A;
    wrt_buff[1] = 0x00;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1);	//disable i2c master mode

    wrt_buff[0] = 0x37;
    wrt_buff[1] = 0x02;
    I2CWriteBytes(imu_fd, I2C_IMU_ADDRESS, wrt_buff, 1);	// bypass multiplexer to config magnetometer

    wrt_buff[0] = 0x0A;
    wrt_buff[1] = 0x1F;
    I2CWriteBytes(imu_fd, I2C_MAG_ADDRESS,  wrt_buff, 1);	// mag fuse ROM access mode
    usleep(10000);

    char temp[1];
    I2CReadBytes(imu_fd, I2C_MAG_ADDRESS, 0x0A, temp, 1);

    char mag_sens_buff[3];
    I2CReadBytes(imu_fd, I2C_MAG_ADDRESS, 0x10, mag_sens_buff, 3);	// read magnetometer sensitivity adjustment
    mag_sens[0] = ((mag_sens_buff[0]-128)/256.0)+1;
    mag_sens[1] = ((mag_sens_buff[1]-128)/256.0)+1;
    mag_sens[2] = ((mag_sens_buff[2]-128)/256.0)+1;

    wrt_buff[0] = 0x0A;
    wrt_buff[1] = 0x00;
    I2CWriteBytes(imu_fd, I2C_MAG_ADDRESS, wrt_buff, 1);	// mag power down mode
    usleep(10000);

    wrt_buff[0] = 0x0A;
    wrt_buff[1] = 0x16;
    I2CWriteBytes(imu_fd, I2C_MAG_ADDRESS, wrt_buff, 1);	// mag 100HZ continuous mode, 16bit data
    usleep(10000);

    // initialization of BMP280 Barometer
    wrt_buff[0] = 0xF4;
    wrt_buff[1] = 0xFF;
    wrt_buff[2] = 0x14;
    I2CWriteBytes(imu_fd, I2C_BARO_ADDRESS, wrt_buff, 2);	//x16 oversampling

    char baro_comp_buff[24];
    I2CReadBytes(imu_fd, I2C_BARO_ADDRESS, 0x88, baro_comp_buff, 24); 	//read barometer and temperature compensation parameters

    temp_comp_param[0] = (baro_comp_buff[1] << 8) | baro_comp_buff[0];
    temp_comp_param[1] = (baro_comp_buff[3] << 8) | baro_comp_buff[2];
    temp_comp_param[2] = (baro_comp_buff[5] << 8) | baro_comp_buff[4];


    press_comp_param[0] = (baro_comp_buff[7] << 8) | baro_comp_buff[6];
    press_comp_param[1] = (baro_comp_buff[9] << 8) | baro_comp_buff[8];
    press_comp_param[2] = (baro_comp_buff[11] << 8) | baro_comp_buff[10];
    press_comp_param[3] = (baro_comp_buff[13] << 8) | baro_comp_buff[12];
    press_comp_param[4] = (baro_comp_buff[15] << 8) | baro_comp_buff[14];
    press_comp_param[5] = (baro_comp_buff[17] << 8) | baro_comp_buff[16];
    press_comp_param[6] = (baro_comp_buff[19] << 8) | baro_comp_buff[18];
    press_comp_param[7] = (baro_comp_buff[21] << 8) | baro_comp_buff[20];
    press_comp_param[8] = (baro_comp_buff[23] << 8) | baro_comp_buff[22];

///// just for debugging
//temp_comp_param[0] = 27504;
//temp_comp_param[1] = 26435;
//temp_comp_param[2] = -1000;

//press_comp_param[0] = 36477;
//press_comp_param[1] = -10685;
//press_comp_param[2] = 3024;
//press_comp_param[3] = 2855;
//press_comp_param[4] = 140;
//press_comp_param[5] = -7;
//press_comp_param[6] = 15500;
//press_comp_param[7] = -14600;
//press_comp_param[8] = 6000;

}

int InitTimer(long long nsec_period, timer_t timer_id, void (*handler)(int, siginfo_t *, void *)){

    struct sigevent signal_event;
    struct itimerspec timer_spec;
    sigset_t mask;
    struct sigaction signal_action;

    signal_action.sa_flags = SA_SIGINFO;
    signal_action.sa_sigaction = handler;
    sigemptyset(&signal_action.sa_mask);
    if (sigaction(SIGRTMIN+timer_count, &signal_action, NULL) < 0)
        cout << "\nSignal Action Error!: " << strerror(errno);

    sigemptyset(&mask);
    sigaddset(&mask, SIGRTMIN+timer_count);
    if (sigprocmask(SIG_SETMASK, &mask, NULL) < 0)
        cout << "\nMasking Error!: " << strerror(errno);

    signal_event.sigev_notify = SIGEV_SIGNAL;
    signal_event.sigev_signo = SIGRTMIN+timer_count;
    signal_event.sigev_value.sival_ptr = &timer_id;
    if (timer_create(CLOCK_REALTIME, &signal_event, &timer_id) < 0)
        cout << "\nTimer Creation Error!: " << strerror(errno);

    timer_spec.it_value.tv_sec = nsec_period / 1000000000;
    timer_spec.it_value.tv_nsec = nsec_period % 1000000000;
    timer_spec.it_interval.tv_sec = timer_spec.it_value.tv_sec;
    timer_spec.it_interval.tv_nsec = timer_spec.it_value.tv_nsec;
    if (timer_settime(timer_id, 0, &timer_spec, NULL) < 0)
        cout << "\nTimer Setting Error!: " << strerror(errno);

    if (sigprocmask(SIG_UNBLOCK, &mask, NULL) < 0)
        cout << "\nUnblocking Error!: " << strerror(errno);

    timer_count++;
    if (timer_count>=SIGRTMAX){
        cout << "\nNo more timer can be created!";
    }

    return (SIGRTMIN+timer_count -1);
}

void OpenSerialPort(const char* port_num, int &file_descriptor, int vmin_set, int vtime_set){

    if((file_descriptor= open(port_num, O_RDWR | O_NOCTTY))< 0)
        cout << "\nSerial Port " <<port_num<<" Openning Error!!: " << strerror(errno);

    // initialization
    struct termios serial_port_settings;

    tcgetattr(file_descriptor, &serial_port_settings);

    //Baudrate
    cfsetispeed(&serial_port_settings,B115200);
    cfsetospeed(&serial_port_settings,B115200);

    //8N1 Mode
    serial_port_settings.c_cflag &= ~PARENB;
    serial_port_settings.c_cflag &= ~CSTOPB;
    serial_port_settings.c_cflag &= ~CSIZE;
    serial_port_settings.c_cflag |=  CS8;
    serial_port_settings.c_cflag &= ~CRTSCTS;
    serial_port_settings.c_cflag |= CREAD | CLOCAL;
    serial_port_settings.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    serial_port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    serial_port_settings.c_oflag &= ~OPOST;

    serial_port_settings.c_cc[VTIME]    = vtime_set;
    serial_port_settings.c_cc[VMIN]     = vmin_set;
    tcflush(file_descriptor, TCIFLUSH);

    if((tcsetattr(file_descriptor, TCSANOW, &serial_port_settings)) != 0)
        cout << "\nSerial Port Initialization Error!: " << strerror(errno);

}

void CloseSerialPort(int &file_descriptor){
    if (close(file_descriptor) <0) /* Close the serial port */
        cout <<"\nSerial Port Closing Error!: " << strerror(errno);
}

void OpenGpio(int pin_num, int direction, int &file_descriptor){
    char command_buff[64];

    if((file_descriptor = open("/sys/class/gpio/export", O_WRONLY)) < 0)
        cout <<"\nGPIO Exporting Error!";

    int length = snprintf(command_buff, sizeof(command_buff), "%d", pin_num);
    write(file_descriptor, command_buff, length);

    close (file_descriptor);

    snprintf(command_buff, sizeof(command_buff), "/sys/class/gpio/gpio%d/direction", pin_num);

    if((file_descriptor = open(command_buff, O_WRONLY)) < 0)
        cout <<"\nGPIO Direction Setting Error!: " << strerror(errno);

    if (direction == 1)
        write(file_descriptor, "out", 4);
    else if (direction == 0)
        write(file_descriptor, "in", 3);

    close (file_descriptor);

    snprintf(command_buff, sizeof(command_buff), "/sys/class/gpio/gpio%d/value", pin_num);
    if ((file_descriptor = open(command_buff, O_WRONLY)) < 0)
        cout << "\nGPIO-"<<pin_num<<" Openning Error!: " << strerror(errno);

    write(file_descriptor, "1",2);

}

void CloseGpio(int &file_descriptor){

    if (close(file_descriptor) <0)
        cout <<"\nGPIO Closing Error!: " << strerror(errno);
}

void OpenI2CPort(const char* port_num, int &file_descriptor){

    if((file_descriptor = open(port_num, O_RDWR))< 0)
        cout << "\nI2C Port " <<port_num<<" Openning Error!!: " << strerror(errno);
}

void CloseI2CPort(int &file_descriptor){
    if (close(file_descriptor) <0) /* Close the serial port */
        cout <<"\nI2C Port Closing Error!: " << strerror(errno);
}

void I2CWriteBytes(int file_descriptor, char device_address, char* reg_adr_wrt_buf, char bytes){
     /* the first byte of "reg_adr_wrt_buf" is the register adress
      * the rest are the values (in the numbre of "bytes") to be writen in consequitive addresses
      */

    if (ioctl(file_descriptor, I2C_SLAVE, device_address) < 0)
        cout << "\nI2C device: " << device_address <<" Slave Setting Error!";

    if (write(file_descriptor, reg_adr_wrt_buf, bytes+1) != bytes+1)
        cout << "\n I2C Write Error! ";
}

void I2CWriteBytes_2(int file_descriptor, char device_address, char* reg_adr_wrt_buf, char bytes){
     /* the first byte of "reg_adr_wrt_buf" is the register adress
      * the rest are the values (in the numbre of "bytes") to be writen in consequitive addresses
      */

    if (ioctl(file_descriptor, I2C_SLAVE, device_address) < 0)
        cout << "\nI2C device: " << device_address <<" Slave Setting Error!";

    if (write(file_descriptor, reg_adr_wrt_buf, bytes+1) != bytes+1)
        cout << "\n I2C Write Error! ";
}

void I2CReadBytes(int file_descriptor, char device_address, char register_address, char* read_buffer, char bytes){

    if (ioctl(file_descriptor, I2C_SLAVE, device_address) < 0)
        cout << "\nI2C device: " << device_address <<" Slave Setting Error!";

    if (write(file_descriptor, &register_address, 1) != 1)
        cout << "\n I2C Register Address: "<< register_address <<" Write Error! " ;

    if (read(file_descriptor, read_buffer, bytes) != bytes)
        cout << "\n I2C Read Error!";

}

void I2CReadBytes_2(int file_descriptor, char device_address, char register_address, char* read_buffer, char bytes){

    if (ioctl(file_descriptor, I2C_SLAVE, device_address) < 0)
        cout << "\nI2C device: " << device_address <<" Slave Setting Error!";

    if (write(file_descriptor, &register_address, 1) != 1)
        cout << "\n I2C Register Address: "<< register_address <<" Write Error! " ;

    if (read(file_descriptor, read_buffer, bytes) != bytes)
        cout << "\n I2C Read Error!";

}

void open_new_file(){

    sprintf(all_file_path, "all_data_IMU_RGBD_GPS_UWB_%d.bin", file_counter);
    all_file = fopen(all_file_path, "r");
    while (all_file){
        fclose(all_file);
        file_counter++;
        sprintf(all_file_path, "all_data_IMU_RGBD_GPS_UWB_%d.bin", file_counter);
        all_file = fopen(all_file_path, "r");
    }

    all_file = fopen(all_file_path, "w+");
    printf("\nFile Path: %s\n", all_file_path);
    printf("\nFile number %d Has been created\n", file_counter);

    /*sprintf(gps_file_path, "GPS_data_%d.bin", gps_file_counter);
    gps_file = fopen(gps_file_path, "r");
    while (gps_file){
        fclose(gps_file);
        gps_file_counter++;
        sprintf(gps_file_path, "GPS_data_%d.bin", gps_file_counter);
        gps_file = fopen(gps_file_path, "r");
    }

    gps_file = fopen(gps_file_path, "w+");
    printf("\nGPS File Path: %s\n", gps_file_path);
    printf("\nGPS File number %d Has been created\n", gps_file_counter);*/
}

void Stop (int n){

    imu_run   = false;
    //imu_run_2 = false;
    gps_run   = false;
}

void Yei_init(uint32_t updateRate, uint32_t streamDuration, uint32_t initDelay, int IMU_no){
	//save the rate
	//updateRate = update_Rate;
	
	/*synchronize time*/
	std::vector<uint8_t> syncCommand;
	syncCommand.push_back(0xF7); //Binary packet byte
	syncCommand.push_back(0x5F); //set timestamp command
	//command data
	/*Data is a 4 byte value in microseconds*/
	uint16_t syncTime = GetTickCount()*1000;
	appendInt(syncTime,syncCommand);
	//Checksum byte
	syncCommand.push_back(Yei_checksum(syncCommand));
	//Send command
	serial->send(&syncCommand.front(),syncCommand.size());	
	
	/* Set up steaming slots */
	std::vector<uint8_t> streamCommand;
	streamCommand.push_back(0xF7); //Binary packet byte
	streamCommand.push_back(0x50); //Command byte
	//command data
	streamCommand.push_back(0x27); //get corrected accel
	streamCommand.push_back(0x26); //get corrected gyro 
	streamCommand.push_back(0x28); //get magnetometer
	streamCommand.push_back(0x01); //get euler angels
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(Yei_checksum(streamCommand)); //Checksum byte
	//Send command
	serial->send(&streamCommand.front(),streamCommand.size());

	/* Set up stream timing */
	std::vector<uint8_t> timingCommand;
	//Binary packet byte
	timingCommand.push_back(0xF7);
	//Command byte
	timingCommand.push_back(0x52);
	//Command data
	appendInt(updateRate,timingCommand);
	appendInt(streamDuration,timingCommand);
	appendInt(initDelay,timingCommand);
	//Checksum byte
	timingCommand.push_back(Yei_checksum(timingCommand));
	//Send command
	serial->send(&timingCommand.front(),timingCommand.size());
	
	/* Setup response header for the timestamp */
	std::vector<uint8_t> resCommand;
	resCommand.push_back(0xF7); //Binary packet byte
	resCommand.push_back(0xDB); //Command byte
	//command data
	/*Data is a 4 byte bitmask to indicate the data to be returned in the header.
	Only the lowest 7 bits are used*/
	appendInt(0x00000002,resCommand); //flip the timestamp
	//Checksum byte
	resCommand.push_back(Yei_checksum(resCommand));
	//Send command
	serial->send(&resCommand.front(),resCommand.size());	
	
	/*start steaming data*/
	std::vector<uint8_t> startCommand;
	startCommand.push_back(0xF9);	//Binary packet with response header byte
	startCommand.push_back(0x55); 	//Command byte
	startCommand.push_back(0x55);   //Checksum byte
	//Send command
	serial->send(&startCommand.front(),startCommand.size());
	
	//read the response header back so it doesn't put the stream out of alignment
	//(Returns 5 bytes even through specified to return 4)
	uint8_t dummy[5];
	serial->get(dummy,5);
	/*std::string filename=std::string("IMU_Data_%d.txt",IMU_no);
	std::cout<<filename;
	const char *cstr = filename.c_str();
	IMU_Data=fopen(cstr,"w");*/
	//IMU_Data=fopen("imudata.txt","w");
	//reset the stop command just in case
	//Yei_setStop(0);
	//Start the update thread	
	/*int result = pthread_create(&yeiThread, 0, &Yei::callUpdate, this);
	if (result != 0){
		printf("Yei Error: Problem creating update thread. Code: %i\n",result);
		pthread_detach(yeiThread);
	}*/
}

uint8_t Yei_checksum(std::vector<uint8_t> data){
	uint8_t sum_of_elems = std::accumulate(data.begin(),data.end(),0);
	sum_of_elems = sum_of_elems - data[0];
	return sum_of_elems % 256;
}







