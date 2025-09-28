#ifndef SPI_DEVICE_HPP
#define SPI_DEVICE_HPP
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

class SpiDevice {
public:
  SpiDevice(const std::string& dev, uint32_t speed_hz = 8'000'000,
    uint8_t mode = SPI_MODE_0, uint8_t bits = 8)
    : fd_(-1), speed_(speed_hz), mode_(mode), bits_(bits) {
    fd_ = ::open(dev.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("open " + dev + " failed");

    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) throw std::runtime_error("SPI set mode failed");
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_) < 0) throw std::runtime_error("SPI set bits failed");
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) throw std::runtime_error("SPI set speed failed");
  }

  ~SpiDevice() { if (fd_ >= 0) ::close(fd_); }

  // Full-duplex transfer; if you only need to TX, pass rx=nullptr.
  void transfer(const void* tx, void* rx, size_t len) {
    struct spi_ioc_transfer tr {};
    tr.tx_buf = reinterpret_cast<__u64>(tx);
    tr.rx_buf = reinterpret_cast<__u64>(rx);
    tr.len = static_cast<__u32>(len);
    tr.speed_hz = speed_;
    tr.bits_per_word = bits_;
    tr.delay_usecs = 0;
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0)
      throw std::runtime_error("SPI transfer failed");
  }

  // Convenience: write-only
  void write(const void* buf, size_t len) { transfer(buf, nullptr, len); }

private:
  int fd_;
  uint32_t speed_;
  uint8_t mode_;
  uint8_t bits_;
};

#endif // SPI_DEVICE_HPP
