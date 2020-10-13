// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
/**==============================================
 * ?                    ABOUT
 * @author      : Maryam
 * @email       : ms_danesh@hotmail.com
 * @repo        : good-engineer
 * @createdOn   : kinect azure SDK
 * @description : body joint tracking, coordinate conversion, data sending to unreal engine
 *  the app stops tracking when press ctr+c
 *=============================================**/


/**===========================================
* ?                     INFO
typedef union {
! XYZ or array representation of vector.
struct _xyz
{
    float x; /**< X component of a vector
    float y; /**< Y component of a vector 
    float z; /**< Z component of a vector 
} xyz;       /**< X, Y, Z representation of a vector. 
float v[3];  /**< Array representation of a vector.
} k4a_float3_t;
===========================================**/

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <k4a/k4a.h>
#include <k4abt.h> 
#include <string.h> 
#include <sys/types.h> 
#include <winsock2.h>
#include <windows.h> 
#include <marvelmind.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 1024	//Max length of buffer // TODO: rearrange buffer size
#define Port 61557
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

volatile sig_atomic_t stop;
void inthand(int signum) {
    stop = 1;
}

// Get Global position of Kinect using Beacon
k4a_float3_t get_kinect_pos(){
    // TODO: Get Global position of Kinect using Beacon
    k4a_float3_t kinect_pos;
    kinect_pos.v[0] = -1;
    kinect_pos.v[1] = -1;
    kinect_pos.v[2] = -1;


    // Init
    struct MarvelmindHedge * hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        puts ("Error: Unable to create MarvelmindHedge");
        return kinect_pos;
    }
    startMarvelmindHedge (hedge);
    
    // Get the position of mobile beacon
    struct PositionValue position; 
    bool valid = getPositionFromMarvelmindHedge(hedge, &position);
    
    if(valid){
        kinect_pos.v[0] = position.x;
        kinect_pos.v[1] = position.y;
        kinect_pos.v[2] = position.z;
    }

    // Exit
    stopMarvelmindHedge (hedge);
    destroyMarvelmindHedge (hedge);
    
    return kinect_pos;
}

// Send data to unreal engine
int send_data(uint32_t  id, k4abt_skeleton_t sk){ 
   
    WSADATA wsdata;
    SOCKET server_socket;
    SOCKADDR_IN server_info;

    int send_size;
    int count;
    char buffer[BUFLEN];


    printf("\nInitialising Winsock...\n");
    if (WSAStartup(MAKEWORD(2,2),&wsdata) != 0)
	{
		printf("Failed. Error Code : %d\n",WSAGetLastError());
		return 1;
	}
    printf("Initialised.\n");

    memset( &server_info, 0, sizeof(server_info) ); // fill with zero
    // memset( buffer, 0, BUFLEN );

    server_info.sin_family = AF_INET;
    server_info.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_info.sin_port = htons(Port);

    // Create socket
    server_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(server_socket == INVALID_SOCKET) {
        printf("Can not create the socket!\n");
        closesocket(server_socket);
        WSACleanup();
        return 1;
    }
    
     for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++){

        k4a_float3_t position = sk.joints[i].position;
//      k4a_quaternion_t orientation = sk.joints[i].orientation;
        k4abt_joint_confidence_level_t confidence_level = sk.joints[i].confidence_level;
//      printf("Body ID[%u], Joint[%d]: Position[mm] ( %f, %f, %f ); \n",id, i, position.v[0], position.v[1], position.v[2]);
        snprintf(buffer, sizeof(buffer),"Body ID[%u], Joint[%d]: Position[mm] ( %f, %f, %f ); \n", 
                                        id, i, position.v[0], position.v[1], position.v[2]);
        
        send_size = sendto (server_socket, buffer, BUFLEN, 0, (struct sockaddr*) &server_info, sizeof(server_info));
        if(send_size < BUFLEN){
            printf("sendto() error!\n");
            return 1;
        }   
    }

    closesocket(server_socket);
 
    WSACleanup();

    return 0;
}


int main(int argc, char** argv)
{
    printf("-------------------Body Joint Tracking----------------------\n");
    printf("Process has started! you can stop tracking by pressing ctr+c keys!\n");

    signal(SIGINT, inthand);

    /**=======================
     * todo      Get kinect global position using beacon
     *  Implement below function
     *========================**/
    
    // Kinect camera global position
    k4a_float3_t kinect_pos = get_kinect_pos(); 

    // Count connected devices
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0) {
        printf("No kinect azure devices has attached!\n");
        return 1;
    }

    // Configure a stream of 4096x3072 BRGA color data at 30 frames per second
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    // Open the first plugged in Kinect device
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!\n");

    // Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char* serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    printf("Device with %s serial number has successfuly opened!\n", serial);
    free(serial);

    // Start the camera with the given configuration
    VERIFY(k4a_device_start_cameras(device, &config), "Start K4A cameras failed!\n");

    // Sensor calibration
    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration),
        "Get depth camera calibration failed!\n");

    // Intialize body tracker
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    int frame_count = 0;
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;

            printf("Start processing frame %d\n", frame_count);

            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

            k4a_capture_release(sensor_capture);
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            // Get body joints, change coordinate and send data
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                printf("%u bodies are detected!\n", num_bodies);

                for (uint32_t i = 0; i < num_bodies; i++)
                {
                    k4abt_body_t body;
                    VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame failed!");
                    body.id = k4abt_frame_get_body_id(body_frame, i);
                    printf("Body ID: %u\n", body.id);
                    
                    for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
                    {
                        k4a_float3_t position = body.skeleton.joints[i].position;
                        k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
                        k4abt_joint_confidence_level_t confidence_level = body.skeleton.joints[i].confidence_level;

                        printf("Body ID: %d ; Original Joint[%d]: Position[mm] ( %f, %f, %f ); Orientation ( %f, %f, %f, %f); Confidence Level (%d) \n",
                           body.id, i, position.v[0], position.v[1], position.v[2], orientation.v[0], orientation.v[1], orientation.v[2], orientation.v[3], confidence_level);
                        
                        // Convert the position
                        position.v[0] = position.v[0] + kinect_pos.v[0];
                        position.v[1] = position.v[1] + kinect_pos.v[1];
                        position.v[2] = position.v[2] + kinect_pos.v[2];

                        printf("Global Joint[%d]: Position[mm] ( %f, %f, %f ); \n",
                            i, position.v[0], position.v[1], position.v[2]);

                    }

                    // Send data to unreal engine
                    send_data(body.id, body.skeleton);
                }

                k4abt_frame_release(body_frame);
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while (!stop);

    printf("Finished body tracking processing!\n");

    // Shut down the camera when finished with application logic
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);

    // Close the device 
    k4a_device_close(device);

    return 0;
}

