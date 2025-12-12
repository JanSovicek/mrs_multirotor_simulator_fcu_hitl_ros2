#include "serial_api.hpp"
#include <umsg.h>
#include <mrs_lib/mutex.h>
#include <chrono>

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

    if (!ser.connect(dev, baudrate, false))
    {
        RCLCPP_ERROR(node_->get_logger(),"could not open serial port");
        return;
    }
    ser.setBlocking(ser.serial_port_fd_, sizeof(umsg_MessageToTransfer));
    q_lock = std::make_unique<CountingSemaphore>(max_packets_in_q);
    umsg_CRCInit();
}

void SerialApi::calculateDelay(umsg_state_heartbeat_t heartbeat)
{
    // auto curr_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
    auto curr_time = clock_->now();
    auto [start_time, sequential] = mrs_lib::get_mutexed(mutex_sync_time, sync_time_ROS_send, sequence_number);

    rclcpp::Duration diff = rclcpp::Duration::from_nanoseconds((curr_time - start_time).nanoseconds() / 2);

    /*syncTime_R is estimated time of HBT arrival at FCU side expressed in ROS time frame*/
    rclcpp::Time syncTime_R = start_time + diff;
    /*syncTime_F is actual time of HBT arrival at FCU side expressed in FCU time frame*/
    uint32_t syncTime_F = heartbeat.timestamp_arrived;

    if (heartbeat.seq_num == sequential - 1)
    {
        mrs_lib::set_mutexed(mutex_sync_result, std::make_tuple(syncTime_R, syncTime_F), sync_result_);

        double time_difference = static_cast<double>((curr_time - start_time).nanoseconds()) / 1e6;
        RCLCPP_INFO(node_->get_logger(),"[SYNC] curr_time %ld ns, start time was %ld ns, computed delay is %.3f miliseconds", curr_time.nanoseconds(), start_time.nanoseconds(), time_difference);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[SYNC] NOT MATCHING SEQUENCE NUMBERS");
    }
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
}

void SerialApi::timerSync()
{

    RCLCPP_INFO_ONCE(node_->get_logger(),"[SerialApi]: Sync timer spinning");

    auto sequential = mrs_lib::get_mutexed(mutex_sync_time, sequence_number);

    umsg_MessageToTransfer msg;

    msg.s.sync0 = 'M';
    msg.s.sync1 = 'R';
    msg.s.len = UMSG_HEADER_SIZE + sizeof(umsg_state_heartbeat_t) + 1;
    msg.s.state.heartbeat.seq_num = sequential;
    msg.s.state.heartbeat.timestamp_arrived = 0;
    msg.s.msg_class = UMSG_STATE;
    msg.s.msg_type = STATE_HEARTBEAT;
    msg.raw[msg.s.len - 1] = umsg_calcCRC(msg.raw, msg.s.len - 1);

    sequential += 1;

    rclcpp::Time curr_time = clock_->now();
    sendPacket(msg);
    mrs_lib::set_mutexed(mutex_sync_time, std::tuple(curr_time, sequential), std::forward_as_tuple(sync_time_ROS_send, sequence_number));

    return;
}

bool SerialApi::isSynced()
{
    return is_synced_;
}
void SerialApi::startReceiver()
{
    recvThread_ = std::thread([this]
                              { this->Receiver(); });
}

void SerialApi::startSyncTimer()
{
    using namespace std::chrono_literals;

    timer_sync_ = node_->create_wall_timer(1s, std::bind(&SerialApi::timerSync, this));
}

umsg_MessageToTransfer SerialApi::waitForPacket()
{

    q_lock->aquire();
    umsg_MessageToTransfer msg = outQ.front();
    outQ.pop();
    return msg;
}

void SerialApi::sendPacket(umsg_MessageToTransfer &msg)
{
    std::unique_lock lock(serial_mutex_);
    ser.sendCharArray(msg.raw, msg.s.len);
}

void SerialApi::Receiver()
{
    state = WAITING_FOR_SYNC0;
    bool receptionComplete = false;
    RCLCPP_INFO_ONCE(node_->get_logger(), "[SerialApi]: ReceiverActive spinning");
    uint32_t readBytes = 0U; // amount of read bytes
    uint32_t toRead = 0U;    // amount of bytes needed to read
    uint32_t toFlush = 0U;
    while (rclcpp::ok())
    {
        switch (state)
        {
        case WAITING_FOR_SYNC0:
        {

            if (readBytes >= 1U)
            {
                if (recvdMsg.s.sync0 != 'M')
                {
                    RCLCPP_ERROR(node_->get_logger(),"[SerialApi] first is the culprit");
                    toFlush = 1U;
                    goto msg_reset;
                }
                state = WAITING_FOR_SYNC1;
                msg_len = 1U;
            }
            else
            {
                toRead = 1U;
            }
            break;
        }
        case WAITING_FOR_SYNC1:
        {
            if (readBytes >= 2U)
            {
                if (recvdMsg.s.sync1 != 'R')
                {
                    RCLCPP_ERROR(node_->get_logger(),"[SerialApi] second is the culprit");
                    toFlush = 2U;
                    goto msg_reset;
                }
                state = WAITING_FOR_HEADER;
                msg_len = 2;
            }
            else
            {
                toRead = 1U;
            }
            break;
        }
        case WAITING_FOR_HEADER:
        {
            if (readBytes >= UMSG_HEADER_SIZE)
            {
                if (recvdMsg.s.len > sizeof(umsg_MessageToTransfer) || msg_len > recvdMsg.s.len)
                {
                    RCLCPP_ERROR(node_->get_logger(),"[SerialApi] third is the culprit");

                    toFlush = 2U;
                    goto msg_flush;
                }
                msg_len = UMSG_HEADER_SIZE;
                state = WAITING_FOR_PAYLOAD;
            }
            else
            {
                toRead = UMSG_HEADER_SIZE - readBytes;
            }
            break;
        }
        // fall through
        case WAITING_FOR_PAYLOAD:
        {
            if (readBytes >= recvdMsg.s.len)
            {
                if (umsg_calcCRC(recvdMsg.raw, recvdMsg.s.len - 1) != recvdMsg.raw[recvdMsg.s.len - 1])
                {
                    RCLCPP_ERROR(node_->get_logger(),"[SerialApi] forth is the culprit");
                    toFlush = 2U;
                    goto msg_flush;
                }
                msg_len += (recvdMsg.s.len - msg_len);
                receptionComplete = true;
                readBytes = 0U;
            }
            else
            {
                toRead = recvdMsg.s.len - readBytes;
            }
        }
        break;

        default:
            toFlush = 2U;
            goto msg_flush;
            break;
        }

        if (receptionComplete)
        {
            // RCLCPP_INFO(node_->get_logger(),"[SerialApi] packet class %d packet type %d", recvdMsg.s.msg_class, recvdMsg.s.msg_type);
            //  RCLCPP_INFO(node_->get_logger(),"[SerialApi] packet Receive");
            if (recvdMsg.s.msg_class == UMSG_STATE && recvdMsg.s.msg_type == STATE_HEARTBEAT)
            {

                umsg_state_heartbeat_t beat = recvdMsg.s.state.heartbeat;

                calculateDelay(beat);
                if (!is_synced_)
                {
                    is_synced_ = true;
                }
            }
            else
            {

                int num_msgs = q_lock->getVal();
                if (num_msgs < max_packets_in_q)
                {
                    outQ.push(recvdMsg);
                    q_lock->release();
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(),"[SerialApi] queue is full");
                }
            }
            // flush
            toFlush = msg_len;
            goto msg_reset;
        }

        if (toRead > 0)
        {
            // RCLCPP_INFO(node_->get_logger(),"[SerialApi] there are %d bytes to read", toRead);

            uint32_t received = ser.readSerial(recvdMsg.raw + readBytes, toRead);
            // RCLCPP_INFO(node_->get_logger(),"[SerialApi] there was %d bytes received", received);
            readBytes += received;
            toRead -= received;
        }

        continue;

    /*Throw out the header and try to catch the next one*/
    msg_reset:
    msg_flush:
        if (readBytes > toFlush)
        {
            readBytes -= toFlush; // flush the header

            for (uint32_t i = 0U; i < readBytes; i++)
            {
                recvdMsg.raw[i] = recvdMsg.raw[i + toFlush];
            }
        }
        else
        {
            readBytes = 0U;
        }
        msg_len = 0U;
        state = WAITING_FOR_SYNC0;
        toRead = 0U;
        receptionComplete = false;
    }
}