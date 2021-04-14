#ifndef COMMON_POST_H_
#define COMMON_POST_H_

#define ASCBOT_OFF                 0
#define ASCBOT_VOID_DANGER_MODE    1
#define ASCBOT_OBJECT_MODE         2
#define ASCBOT_ROAD_MODE           3
#define ASCBOT_REMOTE_MODE         4
#define IMAGE_DATA_SIZE  (256*128*3)

struct shared_common_st {
    int  written_flag;
    int  work_mode;
    int  exit_flag;
    int  video_flag;
    int  remote_direction;
    int  remote_left_speed;
    int  remote_right_speed;

    int  wheel_speed_written_flag;
    int  wheel_left_speed;
    int  wheel_right_speed;

    int  wheel_force_stop;
    int  ip_written_flag;
    char ip_address[20];

    int  image_send_status;
    int  image_data_written_flag;
    int  image_data_type; // 1 yuv420
    int  image_data_size;
    int  image_data_width;
    int  image_data_height;
    unsigned char image_data[IMAGE_DATA_SIZE];
};

struct PostData {
    int  work_mode;
    int  exit_flag;
    int  ip_get_flag;
    char ip_address[20];
    int  remote_direction;
    int  remote_left_speed;
    int  remote_right_speed;
    int  angle;
    int  direction;
    int  check_object;
    int  wheel_left_speed;
    int  wheel_right_speed;
};

#endif /* COMMON_POST_H_ */
