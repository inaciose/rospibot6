// data trasfer variables
union u_tag {
    uint8_t b[4];
    int32_t i;
};

extern union u_tag pwm_data[8];
extern union u_tag enc_data[8];
extern union u_tag tmp_data[8];

extern bool lidarMotorStatus;
