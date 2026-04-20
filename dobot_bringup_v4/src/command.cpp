#include <dobot_bringup/command.h>
#include <iostream>
#include <chrono>
#include <thread>

namespace {
constexpr uint16_t kRealtimeFrameLen = 1440;
constexpr uint64_t kRealtimeTestValue = 0x0123456789ABCDEFULL;

bool is_realtime_frame_header(const uint8_t *buf, size_t n)
{
    if (n < 56)
        return false;
    const uint16_t len = static_cast<uint16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8));
    if (len != kRealtimeFrameLen)
        return false;

    uint64_t test = 0;
    for (size_t i = 0; i < 8; i++)
    {
        test |= (static_cast<uint64_t>(buf[48 + i]) << (8 * i));
    }
    return test == kRealtimeTestValue;
}

bool recv_realtime_frame(TcpClient &tcp, RealTimeData &out, uint32_t timeout_ms)
{
    uint8_t window[56] = {0};
    uint32_t has_read = 0;
    if (!tcp.tcpRecvExact(window, sizeof(window), has_read, timeout_ms))
    {
        return false;
    }

    // Resync: slide a 56-byte window until we find the (len,test_value) signature.
    // This avoids permanent desynchronization when the connection begins mid-frame
    // or when any bytes were previously dropped.
    size_t scanned = 0;
    while (!is_realtime_frame_header(window, sizeof(window)))
    {
        // Shift left by 1 and read 1 new byte.
        memmove(window, window + 1, sizeof(window) - 1);
        if (!tcp.tcpRecvExact(window + (sizeof(window) - 1), 1, has_read, timeout_ms))
        {
            return false;
        }
        scanned++;
        if (scanned > 8192)
        {
            // Give up and let the caller reconnect.
            return false;
        }
    }

    // Copy the header window we already read.
    memcpy(reinterpret_cast<uint8_t *>(&out), window, sizeof(window));
    // Read the remaining bytes of the frame.
    const uint32_t remaining = static_cast<uint32_t>(sizeof(RealTimeData) - sizeof(window));
    if (!tcp.tcpRecvExact(reinterpret_cast<uint8_t *>(&out) + sizeof(window), remaining, has_read, timeout_ms))
    {
        return false;
    }

    return out.len == kRealtimeFrameLen && out.test_value == kRealtimeTestValue;
}
} // namespace
CRCommanderRos2::CRCommanderRos2(const std::string &ip)
    : current_joint_{}, tool_vector_{}, is_running_(false)
{
    is_running_ = false;
    real_time_data_ = std::make_shared<RealTimeData>();
    real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
    dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
}

CRCommanderRos2::~CRCommanderRos2()
{
    is_running_ = false;
    if (thread_ && thread_->joinable())
    {
        thread_->join();
    }
}

void CRCommanderRos2::getCurrentJointStatus(double *joint)
{
    mutex_.lock();
    memcpy(joint, current_joint_, sizeof(current_joint_));
    mutex_.unlock();
}

void CRCommanderRos2::getToolVectorActual(double *val)
{
    mutex_.lock();
    memcpy(val, tool_vector_, sizeof(tool_vector_));
    mutex_.unlock();
}

void CRCommanderRos2::recvTask()
{
    while (is_running_)
    {
        if (real_time_tcp_->isConnect())
        {
            try
            {
                RealTimeData frame{};
                if (!recv_realtime_frame(*real_time_tcp_, frame, 5000))
                {
                    // Either a timeout or the stream couldn't be resynchronized.
                    // Force a reconnect to re-establish alignment.
                    real_time_tcp_->disConnect();
                    continue;
                }

                std::lock_guard<std::mutex> guard(mutex_);
                *real_time_data_ = frame;
                for (uint32_t i = 0; i < 6; i++)
                {
                    current_joint_[i] = deg2Rad(frame.q_actual[i]);
                }
                memcpy(tool_vector_, frame.tool_vector_actual, sizeof(tool_vector_));
            }
            catch (const TcpClientException &err)
            {
                real_time_tcp_->disConnect();
                std::cout << "tcp recv error :" << std::endl;
            }
        }
        else
        {
            try
            {
                real_time_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {
                std::cout << "tcp recv Error : %s" << std::endl;
                sleep(3);
            }
        }

        if (!dash_board_tcp_->isConnect())
        {
            try
            {
                dash_board_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {

                std::cout << "tcp recv ERROR : %s" << std::endl;
                sleep(3);
            }
        }
    }
}

void CRCommanderRos2::init()
{
    try
    {
        is_running_ = true;
        thread_ = std::unique_ptr<std::thread>(new std::thread(&CRCommanderRos2::recvTask, this));
    }
    catch (const TcpClientException &err)
    {
        std::cout << "Commander : %s" << std::endl;
    }
}
int stringToInt(const std::string& str) {
    return std::atoi(str.c_str());
}
void CRCommanderRos2::doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        uint32_t has_read = 0;
        char buf[1024];
        memset(buf, 0, sizeof(buf));
        auto currentTime = std::chrono::system_clock::now();
        auto currentTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
        auto valueMS = currentTime_ms.time_since_epoch().count();
        std::cout <<"time: "<<valueMS <<"  tcp send cmd :" << cmd << std::endl;

        tcp->tcpSend(cmd, strlen(cmd));

        if (!tcp->tcpRecvUntil(buf, sizeof(buf) - 1, has_read, 5000, ';'))
        {
            throw std::logic_error("dashboard tcp recv timeout");
        }
        buf[std::min<uint32_t>(has_read, sizeof(buf) - 1)] = '\0';
        const char *recv_ptr = buf;
        for (int i = 0; i < 2000;i++)  //赋值
        {
            if (recv_ptr[i] == '{')
            {
                std::string str(recv_ptr); // 将char*类型转为string类型
                std::string result = str.substr(0, i-1); // 使用substr函数截取指定长度的子字符串
                int num = stringToInt(result);
                err_id = num;
                std::cout << "ErrorID: " << result<< std::endl;
            }
            
        }

        std::cout << "tcp recv feedback : " << recv_ptr << std::endl; // FIXME parse the buf may be better
    }
    catch (const std::logic_error &err)
    {
        std::cout << "tcpDoCmd failed " << std::endl;
    }
}


void CRCommanderRos2::doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,std::string &mode_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        uint32_t has_read = 0;
        char buf[1024];
        memset(buf, 0, sizeof(buf));
        auto currentTime = std::chrono::system_clock::now();
        auto currentTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
        auto valueMS = currentTime_ms.time_since_epoch().count();
        std::cout <<"time: "<<valueMS <<"  tcp send cmd :" << cmd << std::endl;
        tcp->tcpSend(cmd, strlen(cmd));

        if (!tcp->tcpRecvUntil(buf, sizeof(buf) - 1, has_read, 5000, ';'))
        {
            throw std::logic_error("dashboard tcp recv timeout");
        }
        buf[std::min<uint32_t>(has_read, sizeof(buf) - 1)] = '\0';
        const char *recv_ptr = buf;
        int pose1 = 0;
        for (int i = 0; i < 2000;i++)  //赋值
        {
            if (recv_ptr[i] == '{')
            {
                std::string str(recv_ptr); // 将char*类型转为string类型
                std::string result = str.substr(0, i-1); // 使用substr函数截取指定长度的子字符串
                int num = stringToInt(result);
                err_id = num;
                std::cout << "ErrorID: " << num<< std::endl;
                pose1 = i;
            }
            if (recv_ptr[i] == '}')
            {
                std::string str(recv_ptr); // 将char*类型转为string类型
                std::string result = str.substr(pose1, i-pose1+1); // 使用substr函数截取指定长度的子字符串
                mode_id = result;
                break;
            }
            
        }
        std::cout << "tcp recv feedback : " << recv_ptr << std::endl; // FIXME parse the buf may be better
    }
    catch (const std::logic_error &err)
    {
        std::cout << "tcpDoCmd failed " << std::endl;
    }
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd_f(this->dash_board_tcp_, cmd.c_str(), err_id,mode_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_)
{
    try
    {
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::isEnable() const
{
    std::lock_guard<std::mutex> guard(mutex_);
    return real_time_data_->robot_mode == 5;
}

bool CRCommanderRos2::isConnected() const
{
    return dash_board_tcp_->isConnect() && real_time_tcp_->isConnect();
}

uint16_t CRCommanderRos2::getRobotMode() const
{
    std::lock_guard<std::mutex> guard(mutex_);
    return static_cast<uint16_t>(real_time_data_->robot_mode);
}

std::shared_ptr<RealTimeData> CRCommanderRos2::getRealData() const
{
    return real_time_data_;
}

RealTimeData CRCommanderRos2::getRealDataSnapshot() const
{
    std::lock_guard<std::mutex> guard(mutex_);
    return *real_time_data_;
}
