#define EOF_PACKET_ID				0x00
#define IMU_PACKET_ID				0x01
#define GPSPVT_PACKET_ID			0x02
#define BARO_PACKET_ID				0x03
#define MAG_PACKET_ID				0x04
#define UTC_PACKET_ID				0x05
#define PS_RGB_FRAME_PACKET_ID 		0x06
#define RS_RGB_FRAME_PACKET_ID 		0x07
#define RS_DEPTH_FRAME_PACKET_ID 	0x08
#define UWB_PACKET_ID 				0x09
#define GPSRAW_PACKET_ID			0x0A
#define IMU_PACKET_ID_2				0x0B
#define IMU_PACKET_ID_3				0x0C
#define IMU_PACKET_ID_4				0x0D
#define IMU_PACKET_ID_5				0x0E
#define BARO_PACKET_ID_2			0x0F
#define BARO_PACKET_ID_3			0x10
#define BARO_PACKET_ID_4			0x11
#define BARO_PACKET_ID_5			0x12
#define MAG_PACKET_ID_2 			0x13
#define MAG_PACKET_ID_3				0x14
#define MAG_PACKET_ID_4				0x15
#define MAG_PACKET_ID_5				0x16


#define MAX_LINE_STRINGS_PER_LINK 	70
#define MAX_CONNECTED_LINKS 		10

typedef double			Double64;
typedef float			Float32;
typedef unsigned char	Uint8;
typedef unsigned int	Uint32;
typedef int				Int32;
typedef unsigned short	Uint16;
typedef short int		Int16;

#pragma pack(1)

typedef struct EmsIcdUwbFrameStruct
{
	Uint8		packet_id;
	Uint8		sensor_id;	// sensor 1 and sensor 2
	Uint8		status;
	Double64	time_tag;		//system time (seconds)
	Uint16 		frame_length;
	Float32		frame[1313];	// specific for frame size configuration (1313 floats for 8.5m)
}	EmsIcdUwbFrame, *EmsIcdUwbFramePtr;

/* Intel Realsense RGB camera data
 * If the resolution of the frame is changed
 * in the code, the size of last elelment 
 * should change accordingly*/
typedef struct EmsIcdRsColorStruct		
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;		//system time (seconds)
	//~ Uint16		frame_width;
	//~ Uint16 		frame_height;
	//~ Uint8		byte_per_pixel;
	Uint8		frame[1280*720*3];	// Width*Height*BytePerPixel
}	EmsIcdRsColor, *EmsIcdRsColorPtr;

/* Intel Realsense Depth camera data
 * If the resolution of the frame is changed
 * in the code, the size of last elelment 
 * should change accordingly*/
typedef struct EmsIcdRsDepthStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;		//system time (seconds)
	//~ Uint16		frame_width;
	//~ Uint16 		frame_height;
	//~ Uint8		byte_per_pixel;
	//~ Float32		depth_scale;
	Uint8		frame[1280*720*2];	// Width*Height*BytePerPixel
}	EmsIcdRsDepth, *EmsIcdRsDepthPtr;

/* PlayStation Camera*/
typedef struct EmsIcdPsCameraStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;		//system time (seconds)
	Uint16		frame_width;
	Uint16 		frame_height;
	Uint8		byte_per_pixel;
	Uint32		pixel_format;
	Uint8		frame[640*480*2];	// specific for PlayStation camera and sellected setting width*height*2
}	EmsIcdPsCamera, *EmsIcdPsCameraPtr;

/* Accelerometer and Gyroscope data */
typedef struct EmsIcdImuStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;		//system time (seconds)
	Double64	gps_time_tag;	//within GPS week (seconds)
	Double64	gyro_x;			//deg/sec
	Double64	gyro_y;			//deg/sec
	Double64	gyro_z;			//deg/sec
	Double64	acc_x;			//m/sec^2
	Double64	acc_y;			//m/sec^2
	Double64	acc_z;			//m/sec^2
}	EmsIcdImu, *EmsIcdImuPtr;

/* Magnetometer data */
typedef struct EmsIcdMagStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;//system time (seconds)
	Double64	roll;
	Double64	pitch;
	Double64	heading;
	Double64	heading_std;
	Float32		mag_x;
	Float32		mag_y;
	Float32		mag_z;
}	EmsIcdMag, *EmsIcdMagPtr;

/* Barometer data */
typedef struct EmsIcdBaroStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;//system time (seconds)
	Double64	pressure;		//hPa
	Double64	altitude;		// m above sea level
	Double64	baro_stdv;
}	EmsIcdBaro, *EmsIcdBaroPtr;

typedef struct EmsIcdUTCTimeStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	system_time_tag;
	Uint32 		year;
	Uint32 		month;
	Uint32 		day;
	Uint32 		hour;
	Uint32 		minute;
	Uint32 		second;
}	EmsIcdUTCTime, *EmsIcdUTCTimePtr;

/* GNSS Position Velocity Time (PVT) )Data Per Epoch */
typedef struct EmsIcdGpsPVTStruct
{
	Uint8		packet_id;
	Uint8		status;
	Double64	time_tag;		//system time (seconds)
	Double64	gps_time_tag;	//within GPS week (seconds)
	Double64	latitude;		//deg
	Double64	latitude_std;	//m
	Double64	longitude;		//deg
	Double64	longitude_std;	//m
	Double64	altitude;		//m
	Double64	altitude_std;	//m
	Double64	speed;			//m/s
	Double64	bearing;		//deg
	Double64	accuracy;		//m
	Double64	vn;				//m/s
	Double64	vn_std;			//m/s
	Double64	ve;				//m/s
	Double64	ve_std;			//m/s
	Double64	vd;				//m/s
	Double64	vd_std;			//m/s

}	EmsIcdGpsPVT, *EmsIcdGpsPVTPtr;

/* Raw GNSS Data Per Epoch
 * The size of this structure is variable. 
 * Last three elements must be commented 
 * in this file, but in the code they exist */
typedef struct EmsRawGNSSStruct
{
	Uint8  		packet_id;
	Uint8  		status;
	Double64    time_tag;
	Double64  	gps_time_tag;
	Uint8  		no_svs;					// Number of visible satellites 
	//~ Uint8   	sat_id[no_svs];
	//~ Double64    range[no_svs];		// m
	//~ Double64    range_rate[no_svs];	// m/s
} EmsRawGNSS, *EmsRawGNSSPtr;


//--> Road Networks Structures
typedef struct RoadNetworkLLineStringStruct
{
	double		lattitude_1;
	double		longitude_1;
	double		lattitude_2;
	double		longitude_2;
	double		from_azimuth;
	int 		layout_category;
	char 		direction_of_travel;
	double		azimuth;
}RoadNetworkLLineString, *RoadNetworkLLineStringPtr;

typedef struct RoadNetworkLinkStruct
{
	unsigned int			link_id;
	unsigned short			num_of_line_strings;
	unsigned short			num_of_connect_links;
	unsigned short			num_of_lanes;
	unsigned short			num_of_to_lanes;
	unsigned short			num_of_from_lanes;
	char 					direction_of_travel;
	unsigned int			connected_links[MAX_CONNECTED_LINKS];
	RoadNetworkLLineString 	line_strings[MAX_LINE_STRINGS_PER_LINK];
//	RoadNetworkLanePtr		lanes;
	double 					from_azimuth;
}	RoadNetworkLink, *RoadNetworkLinkPtr;
