#include <dobot_bringup/tcp_socket.h>

TcpClient::TcpClient(std::string ip, uint16_t port) : fd_(-1), port_(port), ip_(std::move(ip)), is_connected_(false)
{
}

TcpClient::~TcpClient()
{
    close();
}

void TcpClient::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
    }
}

void TcpClient::connect()
{
    if (fd_ < 0)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            throw TcpClientException(toString() + std::string(" socket : ") + strerror(errno));
    }

    sockaddr_in addr = {};

    memset(&addr, 0, sizeof(addr));
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);

    if (::connect(fd_, (sockaddr *)&addr, sizeof(addr)) < 0)
        throw TcpClientException(toString() + std::string(" connect : ") + strerror(errno));
    is_connected_ = true;

    std::cout << "connect successfully  " << toString() << std::endl;
}

void TcpClient::disConnect()
{
    if (is_connected_)
    {
        const int fd = fd_;
        fd_ = -1;
        is_connected_ = false;
        if (fd >= 0)
        {
            ::close(fd);
        }
    }
}

bool TcpClient::isConnect() const
{
    return is_connected_;
}

void TcpClient::tcpSend(const void *buf, uint32_t len)
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    //std::cout << "send : " << buf << std::endl;

    const auto *tmp = (const uint8_t *)buf;
    while (len)
    {
        int err = (int)::send(fd_, tmp, len, MSG_NOSIGNAL);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::send() ") + strerror(errno));
        }
        len -= err;
        tmp += err;
    }
}

bool TcpClient::tcpRecv(void *buf, uint32_t len, uint32_t &has_read, uint32_t timeout)
{
    // Legacy behavior: this was historically used to receive dashboard/ASCII responses
    // terminated with ';'. Keeping this behavior avoids breaking callers.
    return tcpRecvUntil(buf, len, has_read, timeout, ';');
}

bool TcpClient::tcpRecvExact(void *buf, uint32_t len, uint32_t &has_read, uint32_t timeout)
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    auto *out = static_cast<uint8_t *>(buf);
    fd_set read_fds;
    timeval tv = {0, 0};

    has_read = 0;
    while (has_read < len)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        const int sel = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (sel < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        if (sel == 0)
        {
            return false;
        }

        const int rd = static_cast<int>(::read(fd_, out + has_read, len - has_read));
        if (rd < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        if (rd == 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }
        has_read += static_cast<uint32_t>(rd);
    }
    return true;
}

bool TcpClient::tcpRecvUntil(void *buf, uint32_t max_len, uint32_t &has_read, uint32_t timeout, char terminator)
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    auto *out = static_cast<uint8_t *>(buf);
    fd_set read_fds;
    timeval tv = {0, 0};

    has_read = 0;
    while (has_read < max_len)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        const int sel = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (sel < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        if (sel == 0)
        {
            // If we've already received some bytes, treat this as end-of-message.
            // Some dashboard servers don't send an explicit terminator and simply
            // stop sending after a short response.
            return has_read > 0;
        }

        const int rd = static_cast<int>(::read(fd_, out + has_read, max_len - has_read));
        if (rd < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        if (rd == 0)
        {
            // If the peer closes after sending a response, treat it as end-of-message.
            if (has_read > 0)
            {
                disConnect();
                return true;
            }
            disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }

        has_read += static_cast<uint32_t>(rd);
        if (out[has_read - 1] == static_cast<uint8_t>(terminator))
        {
            return true;
        }
    }
    return true;
}

std::string TcpClient::toString()
{
    return ip_ + ":" + std::to_string(port_);
}
