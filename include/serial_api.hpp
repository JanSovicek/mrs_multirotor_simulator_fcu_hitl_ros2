#pragma once
#include <rclcpp/rclcpp.hpp>
#include "serial_port.hpp"
#include <umsg_classes.h>
#include <umsg_state.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <queue>


struct TimeStampEntry {
    uint32_t bytes_left;
    rclcpp::Time timestamp;
};

class CountingSemaphore
{
public:
    CountingSemaphore(int max_count);
    void aquire();
    void release();
    int getVal();

private:
    int count;
    int max_count_ = 1;
    std::mutex mtx;
    std::condition_variable cv;
};

class SerialApi
{
private:
    const int max_packets_in_q = 200;
    std::thread recvThread_;
    std::thread serReadThread_;

    std::mutex mutex_sync_time;
    rclcpp::Time sync_time_simulation_ROS_send;
    rclcpp::Time sync_time_steady_clock_ROS_send;
    uint32_t sequence_number = 0;
    std::mutex mutex_sync_result;
    std::tuple<rclcpp::Time, uint32_t> sync_result_; /*Ros simulation time sync result, FCU time*/
    std::atomic<bool> is_synced_ = false;
    rclcpp::TimerBase::SharedPtr timer_sync_;
    
    SerialPort ser_;
    umsg_MessageToTransfer recvdMsg_;

    std::unique_ptr<CountingSemaphore> q_lock_;
    std::queue<umsg_MessageToTransfer> outQ_;

    /*Ring buffer for storing received data*/
    static constexpr uint32_t RX_BUFFER_SIZE = 5*1024; /*10KB*/
    static constexpr uint32_t RX_BUFFER_10_PERCENT_FULL = RX_BUFFER_SIZE/10;
    static constexpr uint32_t RX_BUFFER_25_PERCENT_FULL = RX_BUFFER_SIZE/4;
    static constexpr uint32_t RX_BUFFER_50_PERCENT_FULL = RX_BUFFER_SIZE/2;
    static constexpr uint32_t RX_BUFFER_75_PERCENT_FULL = RX_BUFFER_SIZE*3/4;
    static constexpr uint32_t RX_BUFFER_90_PERCENT_FULL = RX_BUFFER_SIZE*9/10;
    std::vector<uint8_t> rx_data_ring_buffer_; /*Vector is safer for large heap allocation than raw array*/
    std::vector<rclcpp::Time> rx_timestamp_ring_buffer_; /*To store arrival time of each byte for delay calculation*/

    /*Lock free indices*/
    /*Atomic ensures other threads see the update immediately*/
    std::atomic<uint32_t> rx_head_{0}; /*Written by Reader, Read by Parser*/
    std::atomic<uint32_t> rx_tail_{0}; /*Written by Parser, Read by Reader*/

    /*Condition variable to signal data copied into ring buffer*/
    std::condition_variable cv_; 
    std::mutex cv_mutex_;

    uint32_t bytes_sent_ = 0;
    uint32_t bytes_received_ = 0;

    rclcpp::Time recvd_msg_time_{0, 0, RCL_STEADY_TIME};
    static constexpr uint32_t TIME_BETWEEN_PACKETS_US = 100;

    std::deque<uint8_t> tx_queue_;
    std::mutex tx_serial_mutex_;
    std::thread tx_serial_thread_;

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    static constexpr uint32_t SYNC_WINDOW_SIZE = 10;
    std::deque<double> rtt_buffer_;
    double filtered_delay_ms_ = -1.0; 
    const double alpha = 0.5; // Smoothing factor (0.0 to 1.0)

    void initialize(const rclcpp::Node::SharedPtr& node);

    /*Time synchronisation functions*/
    void timerSync();
    void calculateDelay(umsg_state_heartbeat_response_t heartbeat, const rclcpp::Time& arrival_time_steady);

    /*RIng bUffer functions*/
    uint32_t    ringBufferFull();
    uint32_t    ringBufferFree();
    void        ringBufferPop(uint32_t toRead);
    void        ringBufferPush(uint8_t* chunk_buffer, const uint32_t bytes_from_os, const rclcpp::Time& arrival_timestamp_steady);
    uint8_t*    ringBufferPeekPointer(uint32_t offset_from_tail);
    uint32_t    ringBufferPeekUint32(uint32_t offset_from_tail);
    uint8_t     ringBufferCalcCRC(uint32_t total_len);
    void        ringBufferRemove(uint32_t bytes_to_remove);

    void pushMsgToInternalQueue();
    void pin_to_core(pthread_t thread, int core_id);
    void consumeBytesFromTimeQueue(uint32_t bytes_to_consume);

    /*Thread running functions*/
    void SerialRead();
    void Parser();
    void TxThreadLoop();

public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;

    SerialApi(const rclcpp::Node::SharedPtr& node);
    SerialApi(const rclcpp::Node::SharedPtr& node, std::string dev, int baudrate);

    bool isSynced();
    void startSerialApiThreads();
    void startSyncTimer();
    umsg_MessageToTransfer waitForPacket();
    void sendPacket(umsg_MessageToTransfer &msg);
    uint32_t RosToFcu(const rclcpp::Time &rosTime);
    rclcpp::Time FcuToRos(const uint32_t &FcuTime);
};