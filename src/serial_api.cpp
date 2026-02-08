#include "serial_api.hpp"
#include <umsg.h>
#include <mrs_lib/mutex.h>
#include <chrono>
#include <termios.h>

CountingSemaphore::CountingSemaphore(int max_count)
{
    max_count_ = (max_count);
    count = 0;
};

void CountingSemaphore::aquire()
{
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]
            { return count > 0; });
    --count;
}

void CountingSemaphore::release()
{

    std::unique_lock<std::mutex> lock(mtx);
    if (count < max_count_)
    {
        ++count;
        cv.notify_one();
    }
}

int CountingSemaphore::getVal()
{
    std::unique_lock<std::mutex> lock(mtx);
    return count;
}

void SerialApi::initialize(const rclcpp::Node::SharedPtr& node)
{
    node_  = node;
    clock_ = node_->get_clock();
}

SerialApi::SerialApi(const rclcpp::Node::SharedPtr& node)
{
    this->initialize(node);
}

SerialApi::SerialApi(const rclcpp::Node::SharedPtr& node, std::string dev, int baudrate)
{
    this->initialize(node);

    if (!ser_.connect(dev, baudrate, false))
    {
        RCLCPP_ERROR(node_->get_logger(),"could not open serial port");
        return;
    }
    q_lock_ = std::make_unique<CountingSemaphore>(max_packets_in_q);
    umsg_CRCInit();
}

void SerialApi::calculateDelay(umsg_state_heartbeat_response_t heartbeat, const rclcpp::Time& arrival_time_steady)
{
    auto [start_time_sim, start_time_steady, sequential] = mrs_lib::get_mutexed(mutex_sync_time, sync_time_simulation_ROS_send, sync_time_steady_clock_ROS_send, sequence_number);

    // 1. Check sequence number validity
    if (heartbeat.seq_num != sequential - 1) {
        RCLCPP_ERROR(node_->get_logger(), "[SYNC] Mismatch! Got: %u, Expected: %u", heartbeat.seq_num, sequential - 1);
        return;
    }

    // 2. Calculate current RTT in milliseconds
    double current_rtt_ms = static_cast<double>((arrival_time_steady - start_time_steady).nanoseconds()) / 1e6;

    // 3. Update Sliding Window (last SYNC_WINDOW_SIZE)
    rtt_buffer_.push_back(current_rtt_ms);
    if (rtt_buffer_.size() > SYNC_WINDOW_SIZE) {
        rtt_buffer_.pop_front();
    }

    // 4. Find the minimum in the window (the "best" link performance)
    double min_rtt_ms = *std::min_element(rtt_buffer_.begin(), rtt_buffer_.end());

    // 5. Apply Exponential Filter
    if (filtered_delay_ms_ < 0) {
        filtered_delay_ms_ = min_rtt_ms; // Initialize on first packet
    } else {
        filtered_delay_ms_ = (alpha * min_rtt_ms) + ((1.0 - alpha) * filtered_delay_ms_);
    }

    // 6. Use the filtered delay to calculate syncTime_R
    // Divide by 2 to get one-way latency
    rclcpp::Duration filtered_diff = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(filtered_delay_ms_ * 1e6 / 2.0));
    rclcpp::Time syncTime_R = start_time_sim + filtered_diff;
    uint32_t syncTime_F = heartbeat.timestamp_arrived;

    mrs_lib::set_mutexed(mutex_sync_result, std::make_tuple(syncTime_R, syncTime_F), sync_result_);

    RCLCPP_INFO(node_->get_logger(), "[SYNC] Seq: %u | Curr RTT: %.2fms | Min Window: %.2fms | Filtered: %.2fms", 
                heartbeat.seq_num, current_rtt_ms, min_rtt_ms, filtered_delay_ms_);
}

uint32_t SerialApi::RosToFcu(const rclcpp::Time &rosTime)
{
    /*Get the last HBT arrival time at FCU side - in ROS time and FCU time*/
    auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result_);
    int64_t diff = (rosTime.nanoseconds() - syncTime_R.nanoseconds()) / 1e6;
    int64_t new_stamp = diff + static_cast<int64_t>(syncTime_F);
    return static_cast<uint32_t>(new_stamp);
}

rclcpp::Time SerialApi::FcuToRos(const uint32_t &FcuTime)
{
    /*Get the last HBT arrival time at FCU side - in ROS time and FCU time*/
    auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result_);

    /*Get the difference between the message timestamp and the last HBT timestamp*/
    int64_t diff_F = (static_cast<int64_t>(FcuTime) - static_cast<int64_t>(syncTime_F)) * 1e6;
    /*Convert to duration in seconds*/
    rclcpp::Duration diff_R = rclcpp::Duration::from_nanoseconds(diff_F);

    return syncTime_R + diff_R;
    //return clock_->now();
}

void SerialApi::timerSync()
{

    RCLCPP_INFO_ONCE(node_->get_logger(),"[SerialApi]: Sync timer spinning");

    auto sequential = mrs_lib::get_mutexed(mutex_sync_time, sequence_number);

    umsg_MessageToTransfer msg;

    /*uint8_t rawSync[64] = {
    // --- Header (8 bytes) ---
    0x4D, 0x52, 0x04, 0x06, 0x38, 0x00, 0x00, 0x00,

    // --- Payload (8 bytes) ---
    // timestamp_arrived (4 bytes) + seq_numsync (4 bytes)
    0x00, 0x00, 0x00, 0x00, 0xAA, 0xBB, 0xCC, 0xDD,

    // --- Extra / Padding (48 bytes) ---
    // Filled with zeros to reach 64 bytes total
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };*/

    //memcpy(msg.raw, rawSync, 64);

    msg.s.sync0 = 'M';
    msg.s.sync1 = 'R';
    msg.s.len = UMSG_HEADER_SIZE + sizeof(umsg_state_heartbeat_request_t) + 1;
    msg.s.state.heartbeat_request.seq_num = sequential;
    msg.s.state.heartbeat_request.timestamp_arrived = 0;
    msg.s.msg_class = UMSG_STATE;
    msg.s.msg_type = STATE_HEARTBEAT_REQUEST;
    msg.raw[msg.s.len - 1] = umsg_calcCRC(msg.raw, msg.s.len - 1);

    sequential += 1;

    rclcpp::Time curr_time_simulation = clock_->now();
    rclcpp::Time curr_time_steady = steady_clock_.now();
    sendPacket(msg);
    mrs_lib::set_mutexed(mutex_sync_time, std::tuple(curr_time_simulation, curr_time_steady, sequential), std::forward_as_tuple(sync_time_simulation_ROS_send, sync_time_steady_clock_ROS_send, sequence_number));

    RCLCPP_INFO(node_->get_logger(), "[SerialApi]: Sync time message sent, sequence number %u", msg.s.state.heartbeat_request.seq_num);

    return;
}

bool SerialApi::isSynced()
{
    return is_synced_;
}

void SerialApi::pin_to_core(pthread_t thread, int core_id) 
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
}

void SerialApi::startSerialApiThreads()
{
    /*Allocate Memory (Heap)*/
    rx_data_ring_buffer_.resize(RX_BUFFER_SIZE);
    rx_timestamp_ring_buffer_.resize(RX_BUFFER_SIZE);

    RCLCPP_INFO(node_->get_logger(), "[SerialApi]: rx_ring_buffer initialized");
    
    /*Create threads*/
    serReadThread_ = std::thread([this]{ this->SerialRead(); }); 
    recvThread_ = std::thread([this]{ this->Parser(); });
    tx_serial_thread_= std::thread([this]{ this->TxThreadLoop(); });

    /*Pin threads to different cores*/
    pin_to_core(serReadThread_.native_handle(), 2);
    pin_to_core(recvThread_.native_handle(), 3);

    RCLCPP_INFO(node_->get_logger(), "[SerialApi]: threads started and pinned");
}

void SerialApi::startSyncTimer()
{
    using namespace std::chrono_literals;

    timer_sync_ = node_->create_wall_timer(1s, std::bind(&SerialApi::timerSync, this));
}

umsg_MessageToTransfer SerialApi::waitForPacket()
{
    q_lock_->aquire();
    umsg_MessageToTransfer msg = outQ_.front();
    outQ_.pop();
    return msg;
}

void SerialApi::sendPacket(umsg_MessageToTransfer &msg)
{
    std::unique_lock lock(tx_serial_mutex_);
    tx_queue_.insert(tx_queue_.end(), msg.raw, msg.raw + msg.s.len);
}

void SerialApi::TxThreadLoop() 
{
    std::vector<uint8_t> bulk_buffer;
    bulk_buffer.reserve(512);

    while (rclcpp::ok()) 
    {
        // Wait 100us to accumulate data. 
        // This is SAFE now because we are sending ONE big packet, 
        // so we don't care if the OS wakes us up slightly early or late.
        std::this_thread::sleep_for(std::chrono::microseconds(TIME_BETWEEN_PACKETS_US));

        //if(bytes_sent_>=500000)
        //{
        //    RCLCPP_INFO_ONCE(node_->get_logger(), "[SerialApi]: bytes_sent_: %u", bytes_sent_);
        //    std::unique_lock lock(tx_serial_mutex_);
        //    tx_queue_.clear();
        //    continue;
        //}
        //else
        {
            std::unique_lock lock(tx_serial_mutex_);
            if (tx_queue_.empty()) continue;

            // Move all queued data to a local buffer
            bulk_buffer.assign(tx_queue_.begin(), tx_queue_.end());
            tx_queue_.clear();
        }

        // Send as ONE massive USB transaction.
        // The STM32 will receive 200+ bytes in ONE interrupt.
        // No "Short Packet" disable until the very end.
        uint8_t* raw_ptr = bulk_buffer.data();
        size_t length = bulk_buffer.size();
        ser_.sendCharArray(raw_ptr, length);

        bytes_sent_ += length;
        //if(bytes_sent_%100 == 0)
        //{
        //    RCLCPP_INFO(node_->get_logger(), "[SerialApi]: bytes_sent_: %u", bytes_sent_);
        //}
    }
}

// Helper to calculate buffer fullness
uint32_t SerialApi::ringBufferFull() 
{
    /*Load the HEAD*/
    uint32_t tail = rx_tail_.load(std::memory_order_acquire);
    
    /*Load the TAIL*/
    uint32_t head = rx_head_.load(std::memory_order_acquire);

    uint32_t OccupiedBytes = 0U;

    if (head >= tail)
    {
        OccupiedBytes = head - tail;
    } 
    else 
    {
        OccupiedBytes = RX_BUFFER_SIZE + head - tail;
    }

    if(OccupiedBytes > RX_BUFFER_10_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] Receiver: Ring buffer 10%% full");
    }
    else if(OccupiedBytes > RX_BUFFER_25_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] Receiver:Ring buffer 15%% full");
    }
    else if(OccupiedBytes > RX_BUFFER_50_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] Receiver:Ring buffer 50%% full");
    }
    else if(OccupiedBytes > RX_BUFFER_75_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] Receiver:Ring buffer 75%% full");
    }
    else if(OccupiedBytes > RX_BUFFER_90_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] Receiver:Ring buffer 90%% full");
    }
    else
    {
        /*No action*/
    }
    
    return OccupiedBytes;
}

// Helper to calculate buffer free space
uint32_t SerialApi::ringBufferFree() 
{
    /*Load the HEAD*/
    uint32_t head = rx_head_.load(std::memory_order_acquire);

    /*Load the TAIL */
    uint32_t tail = rx_tail_.load(std::memory_order_acquire);

    uint32_t FreeBytes = 0;

    /*-1 is used as a safety gap*/
    if (head >= tail)
    {
        FreeBytes = RX_BUFFER_SIZE - head + tail - 1;
    } 
    else 
    {
        FreeBytes = tail - head - 1;
    }

    if(FreeBytes > RX_BUFFER_90_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] SerialRead:Ring buffer 90%% free");
    }
    else if(FreeBytes > RX_BUFFER_75_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] SerialRead:Ring buffer 75%% free");
    }
    else if(FreeBytes > RX_BUFFER_50_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] SerialRead:Ring buffer 50%% free");
    }
    else if(FreeBytes > RX_BUFFER_25_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] SerialRead:Ring buffer 15%% free");
    }
    else if(FreeBytes > RX_BUFFER_10_PERCENT_FULL)
    {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "[SerialApi] SerialRead:Ring buffer 10%% free");
    }
    else
    {
        /*No action*/
    }

    return FreeBytes;
}

void SerialApi::ringBufferPush(uint8_t* chunk_buffer, const uint32_t bytes_from_os, const rclcpp::Time& arrival_timestamp_steady) 
{
    /*Check space (Atomic Load happens inside ringBufferFree)*/
    uint32_t FreeSpace = ringBufferFree();

    if (bytes_from_os <= FreeSpace) 
    {
        /* Load the HEAD (We own this, so 'relaxed' is fine) */
        uint32_t head = rx_head_.load(std::memory_order_relaxed);
        
        /* Calculate how much continuous space is available until the end of the buffer */
        uint32_t space_until_wrap = RX_BUFFER_SIZE - head;

        if (bytes_from_os <= space_until_wrap) 
        {
            /*No wrapping needed. Just one linear copy.*/
            std::memcpy(&rx_data_ring_buffer_[head], chunk_buffer, bytes_from_os);
            /*Fill in timestamp*/
            std::fill_n( &rx_timestamp_ring_buffer_[head], bytes_from_os, arrival_timestamp_steady);
            
            /*Update local head variable*/
            head += bytes_from_os;

            /*Handle the edge case where we fill exactly to the end*/
            if (head == RX_BUFFER_SIZE) head = 0;
        } 
        else 
        {
            /*Wrapping needed. Two copies.*/
            
            /*Copy Part A: From current head to the end of the buffer*/
            std::memcpy(&rx_data_ring_buffer_[head], chunk_buffer, space_until_wrap);
            std::fill_n( &rx_timestamp_ring_buffer_[head], space_until_wrap, arrival_timestamp_steady);
            
            /*Copy Part B: The remaining bytes to the start of the buffer*/
            uint32_t remaining_bytes = bytes_from_os - space_until_wrap;
            std::memcpy(&rx_data_ring_buffer_[0], &chunk_buffer[space_until_wrap], remaining_bytes);
            std::fill_n( &rx_timestamp_ring_buffer_[0], remaining_bytes, arrival_timestamp_steady);
            
            /*Update local head variable*/
            head = remaining_bytes;
        }

        /* Commit the new head position so Parser sees it */
        /* 'release' ensures the data writes above are visible before the index updates */
        rx_head_.store(head, std::memory_order_release); 
    }
    else 
    {
        /*Rate-limit this error log so it doesn't flood/freeze the CPU if overflow happens*/
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                              "[SerialApi] Ring buffer overflow! Dropped %u bytes", bytes_from_os);
    }
}

/*To be called by Receiver only*/
void SerialApi::ringBufferPop(uint32_t toRead) 
{
    uint32_t current_tail = rx_tail_.load(std::memory_order_relaxed);

    /*Calculate bytes until end of buffer*/
    uint32_t bytes_until_wrap = RX_BUFFER_SIZE - current_tail;

    if (toRead <= bytes_until_wrap) 
    {
        /*Linear copy (No wrap)*/
        memcpy(&recvdMsg_.raw[0], &rx_data_ring_buffer_[current_tail], toRead);
        /*Get timestamp od last Msg byte*/
        recvd_msg_time_ = rx_timestamp_ring_buffer_[current_tail+toRead-1];

        rx_tail_.store((current_tail + toRead) % RX_BUFFER_SIZE, std::memory_order_release);
    } 
    else 
    {
        /*Wrapped copy (Part 1: Tail to End, Part 2: Start to Rest)*/
        memcpy(&recvdMsg_.raw[0], &rx_data_ring_buffer_[current_tail], bytes_until_wrap);
        
        uint32_t remaining = toRead - bytes_until_wrap;
        memcpy(&recvdMsg_.raw[bytes_until_wrap], &rx_data_ring_buffer_[0], remaining);
        /*Get timestamp od last Msg byte*/
        recvd_msg_time_ = rx_timestamp_ring_buffer_[remaining-1];
        
        rx_tail_.store(remaining, std::memory_order_release);
    }

    return;
}

uint8_t* SerialApi::ringBufferPeekPointer(uint32_t offset_from_tail)
{
    uint32_t current_tail = rx_tail_.load(std::memory_order_relaxed);
    return &rx_data_ring_buffer_[(current_tail+offset_from_tail)%RX_BUFFER_SIZE];
}

uint32_t SerialApi::ringBufferPeekUint32(uint32_t offset_from_tail) 
{
    uint32_t value = 0;
    uint32_t index = (rx_tail_.load(std::memory_order_relaxed)+offset_from_tail) % RX_BUFFER_SIZE; // Locate start byte
    
    // FAST PATH: Data is contiguous (No wrap)
    // We check if the 4 bytes fit before the end of the physical array
    if (index + 4 <= RX_BUFFER_SIZE) {
        memcpy(&value, &rx_data_ring_buffer_[index], 4); 
    } 
    // SLOW PATH: Data wraps around the end
    else 
    {
        // Reconstruct byte-by-byte
        uint8_t b0 = rx_data_ring_buffer_[index];
        uint8_t b1 = rx_data_ring_buffer_[(index + 1) % RX_BUFFER_SIZE];
        uint8_t b2 = rx_data_ring_buffer_[(index + 2) % RX_BUFFER_SIZE];
        uint8_t b3 = rx_data_ring_buffer_[(index + 3) % RX_BUFFER_SIZE];

        // Combine (Little Endian)
        value = (uint32_t)b0 | 
               ((uint32_t)b1 << 8) | 
               ((uint32_t)b2 << 16) | 
               ((uint32_t)b3 << 24);
    }
    
    return value;
}

uint8_t SerialApi::ringBufferCalcCRC(uint32_t total_len) 
{
  // 1. Get Start Index and Pointers
  uint32_t tail = rx_tail_.load(std::memory_order_relaxed);
  
  uint8_t crc = 0; // Initial seed
  
  uint32_t chunk1_len;

  // set chunk 1
  if (tail + total_len <= RX_BUFFER_SIZE) 
  {
    chunk1_len = total_len;
  }
  else
  {
    chunk1_len = RX_BUFFER_SIZE-tail;
  }

  // Process the First Chunk
  // We pass '0' as the initial remainder for the start of the packet
  crc = umsg_calcCRC_split(&rx_data_ring_buffer_[tail], chunk1_len, 0);

  // Process the Wrap-Around Chunk (If needed)
  if (total_len > chunk1_len) 
  {
    uint32_t chunk2_len = total_len - chunk1_len;
    //Pass the 'crc' result from Step 1 as the 'remainder' input here
    crc = umsg_calcCRC_split((&rx_data_ring_buffer_[0]), chunk2_len, crc);
  }

  return crc;
}

void SerialApi::ringBufferRemove(uint32_t bytes_to_remove)
{
  uint32_t new_tail = (rx_tail_.load(std::memory_order_relaxed)+bytes_to_remove)%RX_BUFFER_SIZE;
  rx_tail_.store(new_tail, std::memory_order_release);
}

void SerialApi::pushMsgToInternalQueue()
{
    // RCLCPP_INFO(node_->get_logger(),"[SerialApi] packet class %d packet type %d", recvdMsg_.s.msg_class, recvdMsg_.s.msg_type);
    //  RCLCPP_INFO(node_->get_logger(),"[SerialApi] packet Receive");
    if (recvdMsg_.s.msg_class == UMSG_STATE && recvdMsg_.s.msg_type == STATE_HEARTBEAT_RESPONSE)
    {
        /*Process heart beat and calculate time delay*/
        umsg_state_heartbeat_response_t beat = recvdMsg_.s.state.heartbeat_response;

        //rclcpp::Time current_packet_arrival_time = steady_clock_.now();

        calculateDelay(beat, recvd_msg_time_);
        if (!is_synced_)
        {
            is_synced_ = true;
        }
    }
    else if (recvdMsg_.s.msg_class == UMSG_STATE && recvdMsg_.s.msg_type == STATE_HEARTBEAT_REQUEST)
    {
        RCLCPP_ERROR(node_->get_logger(),"[SerialApi] Echo read");
    }
    else
    {
        /*Push message into queue*/
        int num_msgs = q_lock_->getVal();
        if (num_msgs < max_packets_in_q)
        {
            outQ_.push(recvdMsg_);
            q_lock_->release();
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),"[SerialApi] queue is full");
        }
    }
}

void SerialApi::Parser()
{
    RCLCPP_INFO_ONCE(node_->get_logger(), "[SerialApi]: Receiver Active spinning");
        
    while (rclcpp::ok())
    {
        {
            std::unique_lock<std::mutex> lock(cv_mutex_);

            /*Check if data arrived while we were grabbing the lock*/
            /*Since we hold the lock now, the Reader cannot be in the middle of a notification*/
            if (ringBufferFull() <= UMSG_HEADER_SIZE) // Or just > 0
            {  
                /*Wait until serial reader notifies OR timeout (safety)*/
                cv_.wait(lock); 
                continue; /*Wake up and check number of bytes*/
            }
        }

        uint8_t* pRingBuf = ringBufferPeekPointer(0);

        if(*pRingBuf == 'M')
        {
            pRingBuf = ringBufferPeekPointer(1);
            if(*pRingBuf == 'R')
            {
                uint32_t len = ringBufferPeekUint32(4);

                if ((len < UMSG_HEADER_SIZE + 1) || (len > sizeof(umsg_MessageToTransfer)))
                {
                    /*Invalid length - remove data and start again*/
                    ringBufferRemove(2);
                    continue;
                }

                while(1)
                {
                    std::unique_lock<std::mutex> lock(cv_mutex_);

                    if (ringBufferFull()>= len) 
                    {
                        break; // We have enough data to process the message
                    }
                    else
                    {
                        cv_.wait(lock);
                        continue;
                    }
                }

                pRingBuf = ringBufferPeekPointer(len-1); //CRC position
                if (ringBufferCalcCRC(len-1) == *pRingBuf)
                {
                    /*Valid CRC - copy message and remove it from ring buffer*/
                    ringBufferPop(len);
                    pushMsgToInternalQueue();
                }
                else
                {
                    ringBufferRemove(2);
                }
            }
            else
            {
                ringBufferRemove(1);
            }
        }
        else
        {
            ringBufferRemove(1);
        }
    }
    return;
}

void SerialApi::SerialRead()
{
    /*Temporary buffer for reading from OS*/
    static constexpr uint32_t CHUNK_BUFFER_SIZE = 1024;
    uint8_t chunk_buffer[CHUNK_BUFFER_SIZE];

    RCLCPP_INFO_ONCE(node_->get_logger(), "[SerialApi]: SerialRead Active spinning");
    
    /*Flush data received on the serial port*/
    // TCIFLUSH: Flushes data received but not read
    // TCOFLUSH: Flushes data written but not transmitted
    // TCIOFLUSH: Flushes both
    //tcflush(ser_.serial_port_fd_, TCIFLUSH);

    //uint32_t n = 1;

    while(rclcpp::ok())
    {
        /*Blocking read*/
        uint32_t bytes_from_os = ser_.readSerial(chunk_buffer, CHUNK_BUFFER_SIZE);
        auto arrival_time = steady_clock_.now();

        if(0 < bytes_from_os)
        {
            ringBufferPush(chunk_buffer, bytes_from_os, arrival_time);
            bytes_received_ += bytes_from_os;
            //if(bytes_received_%2 == 0)
            //{
            //    RCLCPP_INFO(node_->get_logger(), "[SerialApi] SerialRead: bytes_received_: %u", bytes_received_);
            //} 

            /*Notify the parser*/
            {
                cv_.notify_one();
            }    
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),"[SerialApi] Error while reading serial, try again");
        }
        
    }
}
