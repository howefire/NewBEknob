#pragma once


class Logger {
    public:
        bool output_io = true;
        uint8_t Recv_buf[120];
        size_t rec_len =0;
        uint8_t sned_len;
        uint32_t temp_data = 0;
        uint8_t start;
        uint8_t end;
        uint8_t send_buf[20];
        char num_data[40];
        Logger() {};
        virtual ~Logger() {};
        virtual void log(const char* msg) = 0;
};