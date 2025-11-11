#include "serial_api.hpp"
#include <umsg.h>
#include <mrs_lib/mutex.h>

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

SerialApi::SerialApi(/* args */)
{
}
SerialApi::SerialApi(std::string dev, int baudrate)
{

    if (!ser.connect(dev, baudrate, false))
    {
        RCLCPP_ERROR(ser_api_logger,"could not open serial port");
        return;
    }
    ser.setBlocking(ser.serial_port_fd_, sizeof(umsg_MessageToTransfer));
    q_lock = std::make_unique<CountingSemaphore>(max_packets_in_q);
    umsg_CRCInit();
}

void SerialApi::calculateDelay(umsg_state_heartbeat_t heartbeat)
{
    node_  = node;
    clock_ = node_->get_clock();
    // auto curr_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
    auto curr_time = rclcpp::Time::now();
    auto [start_time, sequential] = mrs_lib::get_mutexed(mutex_sync_time, sync_time_ROS_send, sequence_number);

    rclcpp::Duration diff = rclcpp::Duration::from_nanoseconds((curr_time - start_time).nanoseconds() / 2);

    rclcpp::Time syncTime_R = start_time + diff;
    uint32_t syncTime_F = heartbeat.timestamp_arrived;

    if (heartbeat.seq_num == sequential - 1)
    {
        mrs_lib::set_mutexed(mutex_sync_result, std::make_tuple(syncTime_R, syncTime_F), sync_result);

        double time_difference = static_cast<double>((curr_time - start_time).toNSec()) / 1e6;
        ROS_INFO("[SYNC] curr_time %ld start was %ld delay was %.3f miliseconds", curr_time.toNSec(), start_time.toNSec(), time_difference);
    }
    else
    {
        ROS_ERROR("[SYNC] NOT MATCHING SEQUENCE NUMBERS");
    }
}

uint32_t SerialApi::RosToFcu(const ros::Time &rosTime)
{

    auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
    int64_t diff = (rosTime.toNSec() - syncTime_R.toNSec()) / 1e6;
    int64_t new_stamp = diff + static_cast<int64_t>(syncTime_F);
    return static_cast<uint32_t>(new_stamp);
}

ros::Time SerialApi::FcuToRos(const uint32_t &FcuTime)
{
    auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
    int64_t diff = (static_cast<int64_t>(FcuTime) - static_cast<int64_t>(syncTime_F)) * 1e6;

    ros::Duration diff_R;
    diff_R.fromNSec(diff);

    return syncTime_R + diff_R;
}

void SerialApi::timerSync(const ros::WallTimerEvent &event)
{

    ROS_INFO_ONCE("[SerialApi]: sync timer spinning");

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

    // auto curr_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
    auto curr_time = ros::Time::now();
    sendPacket(msg);
    // auto curr_time   = ros::time::now();
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

void SerialApi::startSyncTimer(ros::NodeHandle &nh_)
{
    timer_sync_ = nh_.createWallTimer(ros::WallDuration(1.00), &SerialApi::timerSync, this);
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
    ROS_INFO_ONCE("[SerialApi]: ReceiverActive spinning");
    int readBytes = 0; // amount of read bytes
    int toRead = 0;    // amount of bytes needed to read
    int toFlush = 0;
    while (true)
    {
        switch (state)
        {
        case WAITING_FOR_SYNC0:
        {

            if (readBytes >= 1)
            {
                if (recvdMsg.s.sync0 != 'M')
                {
                    ROS_ERROR("first is the culprit");
                    toFlush = 1;
                    goto msg_reset;
                }
                state = WAITING_FOR_SYNC1;
                msg_len = 1;
            }
            else
            {
                toRead = 1;
            }
            break;
        }
        case WAITING_FOR_SYNC1:
        {
            if (readBytes >= 2)
            {
                if (recvdMsg.s.sync1 != 'R')
                {
                    ROS_ERROR("second is the culprit");
                    toFlush = 2;
                    goto msg_reset;
                }
                state = WAITING_FOR_HEADER;
                msg_len = 2;
            }
            else
            {
                toRead = 1;
            }
            break;
        }
        case WAITING_FOR_HEADER:
        {
            if (readBytes >= UMSG_HEADER_SIZE)
            {
                if (recvdMsg.s.len > sizeof(umsg_MessageToTransfer) || msg_len > recvdMsg.s.len)
                {
                    ROS_ERROR("third is the culprit");

                    toFlush = 2;
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
                    ROS_ERROR("forth is the culprit");
                    toFlush = 2;
                    goto msg_flush;
                }
                msg_len += (recvdMsg.s.len - msg_len);
                receptionComplete = true;
                readBytes = 0;
            }
            else
            {
                toRead = recvdMsg.s.len - readBytes;
            }
        }
        break;

        default:
            toFlush = 2;
            goto msg_flush;
            break;
        }

        if (receptionComplete)
        {
            // ROS_INFO("[SerialApi] packet class %d packet type %d", recvdMsg.s.msg_class, recvdMsg.s.msg_type);
            //  ROS_INFO("[SerialApi] packet Receive");
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
                    ROS_ERROR("[SerialApi] queue is full");
                }
            }
            // flush
            toFlush = msg_len;
            goto msg_reset;
        }

        if (toRead > 0)
        {
            // ROS_INFO("[SerialApi] there are %d bytes to read", toRead);

            int received = ser.readSerial(recvdMsg.raw + readBytes, toRead);
            // ROS_INFO("[SerialApi] there was %d bytes received", received);
            readBytes += received;
            toRead += -received;
        }

        continue;

    // throw out the header and try to catch the next one
    msg_reset:
    msg_flush:
        readBytes += -toFlush; // flush the header
        if (readBytes >= 0)
        {
            for (size_t i = 0; i < readBytes; i++)
            {
                recvdMsg.raw[i] = recvdMsg.raw[i + toFlush];
            }
        }
        else
        {
            readBytes = 0;
        }
        msg_len = 0;
        state = WAITING_FOR_SYNC0;
        toRead = 0;
        receptionComplete = false;
    }
}