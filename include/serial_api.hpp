#pragma once
#include <rclcpp/rclcpp.hpp>
#include "serial_port.hpp"
#include <umsg_classes.h>
#include <umsg_state.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <queue>

enum ReceiverState
{
    WAITING_FOR_SYNC0,
    WAITING_FOR_SYNC1,
    WAITING_FOR_HEADER,
    WAITING_FOR_PAYLOAD,
    INVALID_MSG,
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

    std::mutex mutex_sync_time;
    rclcpp::Time sync_time_ROS_send;
    uint32_t sequence_number = 0;
    std::mutex mutex_sync_result;
    std::tuple<rclcpp::Time, uint32_t> sync_result;
    std::atomic<bool> is_synced_ = false;
    rclcpp::TimerBase::SharedPtr timer_sync_;
    
    std::mutex serial_mutex_;
    SerialPort ser;
    umsg_MessageToTransfer recvdMsg;
    uint32_t msg_len = 0;
    std::unique_ptr<CountingSemaphore> q_lock;
    std::queue<umsg_MessageToTransfer> outQ;
    ReceiverState state = WAITING_FOR_SYNC0;

    void initialize(const rclcpp::Node::SharedPtr& node);
    void timerSync();
    void calculateDelay(umsg_state_heartbeat_t heartbeat);
    void Receiver();

public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;

    SerialApi(const rclcpp::Node::SharedPtr& node);
    SerialApi(const rclcpp::Node::SharedPtr& node, std::string dev, int baudrate);

    bool isSynced();
    void startReceiver();
    void startSyncTimer();
    umsg_MessageToTransfer waitForPacket();
    void sendPacket(umsg_MessageToTransfer &msg);
    uint32_t RosToFcu(const rclcpp::Time &rosTime);
    rclcpp::Time FcuToRos(const uint32_t &FcuTime);
};