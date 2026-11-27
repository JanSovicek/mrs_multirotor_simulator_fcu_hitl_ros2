#pragma once
#include <rclcpp/rclcpp.hpp>
#include "serial_port.hpp"
#include <umsg_classes.h>
#include <umsg_state.h>
#include <umsg_debug.h>
#include <array>
#include <atomic>
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
    const int max_packets_in_q = 2000;
    std::thread recvThread_;
    std::thread serReadThread_;

    std::mutex mutex_sync_time;
    rclcpp::Time sync_time_simulation_ROS_send;
    rclcpp::Time sync_time_steady_clock_ROS_send;
    uint32_t sequence_number = 0;
    rclcpp::TimerBase::SharedPtr timer_sync_;
    
    SerialPort ser_;
    umsg_MessageToTransfer recvdMsg_;

    /*Reconnect after an FCU reset (USB re-enumeration). SerialRead owns the port lifetime: it detects the disconnect,
      closes the port and reopens it periodically. The TX thread writes only under port_mutex_ while connected*/
    std::string dev_;
    int         baudrate_ = 0;
    std::mutex  port_mutex_;                          /*serialises write() against close()/connect()*/
    std::atomic<bool> port_connected_{false};
    /*Set by SerialRead on a disconnect, applied and cleared by the Parser thread (owner of rx_tail_, the RTT gate and the
      LinkStats state): RX ring flush, time sync and LinkStats reset. The port is reopened only after it was applied*/
    std::atomic<bool> link_reset_request_{false};
    static constexpr std::chrono::milliseconds RECONNECT_PERIOD{500};
    /*A lucky heartbeat whose offset differs from the filtered one by more than this is a clock restart (FCU reboot) - the
      offset is taken directly instead of being averaged in over tens of seconds*/
    static constexpr int64_t OFFSET_JUMP_REINIT_NS = 100000000; /*100 ms*/

    void handleDisconnect();
    bool tryReconnect();
    void applyLinkReset();

    std::unique_ptr<CountingSemaphore> q_lock_;
    std::queue<umsg_MessageToTransfer> outQ_;
    std::mutex outQ_mutex_; /*Protects outQ_ - pushed by the Parser thread, popped by the messageParser thread*/

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
    static constexpr std::chrono::milliseconds PARSER_WAIT_TIMEOUT{100}; /*Parser wake-up period without data (shutdown check)*/

    uint32_t bytes_sent_ = 0;
    uint32_t bytes_received_ = 0;

    rclcpp::Time recvd_msg_time_{0, 0, RCL_STEADY_TIME};
    static constexpr uint32_t TIME_BETWEEN_PACKETS_US = 100;

    std::deque<uint8_t> tx_queue_;
    std::mutex tx_serial_mutex_;
    std::thread tx_serial_thread_;

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    /*PC->FCU link accounting (LinkStats). The PC records its TX stream position right after each HeartbeatRequest, the FCU
      reports how many bytes its parser consumed up to and including that request - the difference is the exact number of
      bytes that never made it through (no in-flight uncertainty, USB CDC is ordered)*/
    uint64_t tx_stream_bytes_ = 0;               /*bytes queued for TX since start - protected by tx_serial_mutex_*/
    std::atomic<uint64_t> tx_failed_bytes_{0};   /*bytes of bulk writes that failed (upper bound - partial writes unknown)*/

    struct LinkTxMark
    {
        uint32_t seq_num  = 0;
        uint64_t tx_bytes = 0;     /*TX stream position at the end of the HeartbeatRequest frame*/
        bool     valid    = false;
    };
    static constexpr size_t LINK_TX_MARKS = 32;  /*heartbeat 1 Hz - 32 s of outstanding requests*/
    std::array<LinkTxMark, LINK_TX_MARKS> link_tx_marks_{};

    /*Counters at the baseline (first LinkStats or FCU reboot) and at the previous report - Parser thread only*/
    struct LinkCounters
    {
        uint64_t pc_tx_bytes       = 0;
        uint64_t pc_tx_failed      = 0;
        uint64_t fcu_rx_bytes      = 0;
        uint64_t fcu_overflow      = 0;
        uint64_t fcu_garbage       = 0;
        uint64_t fcu_rejected_ts   = 0;
        uint64_t fcu_rejected_val  = 0;
    };
    std::mutex   link_mutex_;                    /*protects link_tx_marks_*/
    bool         link_baseline_valid_ = false;
    LinkCounters link_baseline_{};
    LinkCounters link_last_{};
    int64_t      link_last_missing_ = 0;

    void handleLinkStats(const umsg_debug_LinkStats_t& stats);

    /*Lucky-packet gate reference: minimum RTT over the last SYNC_WINDOW_SIZE heartbeats (1 Hz -> 10 s), all heartbeats
      enter the window (accepted or not), so a once-in-a-lifetime low RTT ages out instead of rejecting everything forever*/
    static constexpr uint32_t SYNC_WINDOW_SIZE = 10;
    std::deque<double> rtt_buffer_;
    double filtered_delay_ms_ = -1.0;

    const double alpha = 0.1; // Smoothing factor (0.0 to 1.0)
    double historical_min_rtt_ = 9999.0;              /*min of rtt_buffer_ - Parser thread only*/

    /*Offset jump confirmation (two-sample, as TimeStepGuard): a lucky heartbeat whose offset jumps by more than
      OFFSET_JUMP_REINIT_NS only becomes a candidate, the offset is re-initialized when the next lucky heartbeat agrees with
      the candidate within OFFSET_JUMP_CONFIRM_NS. A heartbeat agreeing with the old offset drops the candidate*/
    static constexpr int64_t OFFSET_JUMP_CONFIRM_NS = 10000000; /*10 ms*/
    bool    offset_jump_pending_      = false;
    int64_t offset_jump_candidate_ns_ = 0;
    std::atomic<int64_t> smoothed_offset_ns_{0};
    std::atomic<bool> offset_initialized_{false};

    void initialize(const rclcpp::Node::SharedPtr& node);

    /*Time synchronisation functions*/
    void timerSync();
    void calculateDelay(umsg_state_HeartbeatResponse_t heartbeat, const rclcpp::Time& arrival_time_steady);

    /*RIng bUffer functions*/
    uint32_t    ringBufferFull();
    uint32_t    ringBufferFree();
    void        ringBufferPop(uint32_t toRead);
    void        ringBufferPush(uint8_t* chunk_buffer, const uint32_t bytes_from_os, const rclcpp::Time& arrival_timestamp_steady);
    uint8_t*    ringBufferPeekPointer(uint32_t offset_from_tail);
    uint32_t    ringBufferPeekUint32(uint32_t offset_from_tail);
    bool        ringBufferCheckCRC(uint32_t total_len);
    void        ringBufferRemove(uint32_t bytes_to_remove);

    void pushMsgToInternalQueue();
    void pin_to_core(pthread_t thread, int core_id);

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
    /*Returns the TX stream position (bytes queued since start) at the end of this message*/
    uint64_t sendPacket(umsg_MessageToTransfer &msg);
    uint64_t RosToFcu(const rclcpp::Time &rosTime);
    rclcpp::Time FcuToRos(const uint64_t &FcuTime);
};