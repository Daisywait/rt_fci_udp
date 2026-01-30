#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iostream>
#include <string>
#include <thread>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>

namespace {
struct CmdPacket {
  double v[6];
  uint64_t t_ns;  // sender timestamp (optional)
};

struct PosePacket {
  double T[16];   // column-major 4x4, same layout as franka::RobotState::O_T_EE
  uint64_t t_ns;  // sender timestamp
};

struct CmdShared {
  std::atomic<uint64_t> seq{0};
  std::array<double, 6> v{{0, 0, 0, 0, 0, 0}};
  std::atomic<uint64_t> last_rx_ns{0};
};

struct PoseShared {
  std::atomic<uint64_t> seq{0};
  std::array<double, 16> T{};
  std::atomic<uint64_t> t_ns{0};
};

uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool read_cmd(const CmdShared& shared, std::array<double, 6>& out, uint64_t& last_rx_ns) {
  for (;;) {
    uint64_t s1 = shared.seq.load(std::memory_order_acquire);
    if (s1 & 1u) {
      continue;
    }
    out = shared.v;
    last_rx_ns = shared.last_rx_ns.load(std::memory_order_acquire);
    uint64_t s2 = shared.seq.load(std::memory_order_acquire);
    if (s1 == s2 && !(s2 & 1u)) {
      return true;
    }
  }
}

void write_pose(PoseShared& shared, const std::array<double, 16>& T, uint64_t t_ns) {
  uint64_t s = shared.seq.load(std::memory_order_relaxed);
  shared.seq.store(s + 1, std::memory_order_release);
  shared.T = T;
  shared.t_ns.store(t_ns, std::memory_order_release);
  shared.seq.store(s + 2, std::memory_order_release);
}

bool read_pose(const PoseShared& shared, std::array<double, 16>& T, uint64_t& t_ns) {
  for (;;) {
    uint64_t s1 = shared.seq.load(std::memory_order_acquire);
    if (s1 & 1u) {
      continue;
    }
    T = shared.T;
    t_ns = shared.t_ns.load(std::memory_order_acquire);
    uint64_t s2 = shared.seq.load(std::memory_order_acquire);
    if (s1 == s2 && !(s2 & 1u)) {
      return true;
    }
  }
}

int make_udp_receiver(const std::string& bind_ip, int port) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind_ip.empty() || bind_ip == "0.0.0.0") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    if (::inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) != 1) {
      ::close(fd);
      return -1;
    }
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

int make_udp_sender() {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  return fd;
}

void cmd_rx_thread(CmdShared& shared, const std::string& bind_ip, int port) {
  int fd = make_udp_receiver(bind_ip, port);
  if (fd < 0) {
    std::cerr << "Failed to bind UDP cmd receiver on " << bind_ip << ":" << port << "\n";
    return;
  }

  for (;;) {
    CmdPacket pkt{};
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);
    ssize_t n = ::recvfrom(fd, &pkt, sizeof(pkt), 0, reinterpret_cast<sockaddr*>(&src), &srclen);
    if (n == static_cast<ssize_t>(sizeof(pkt))) {
      uint64_t s = shared.seq.load(std::memory_order_relaxed);
      shared.seq.store(s + 1, std::memory_order_release);
      for (int i = 0; i < 6; ++i) {
        shared.v[i] = pkt.v[i];
      }
      shared.last_rx_ns.store(now_ns(), std::memory_order_release);
      shared.seq.store(s + 2, std::memory_order_release);
    }
  }
}

void pose_tx_thread(const PoseShared& shared,
                    const std::string& target_ip,
                    int port,
                    int send_hz) {
  int fd = make_udp_sender();
  if (fd < 0) {
    std::cerr << "Failed to create UDP pose sender\n";
    return;
  }
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, target_ip.c_str(), &dst.sin_addr) != 1) {
    std::cerr << "Invalid target IP: " << target_ip << "\n";
    ::close(fd);
    return;
  }

  const auto period = std::chrono::microseconds(static_cast<int>(1e6 / send_hz));
  for (;;) {
    PosePacket pkt{};
    std::array<double, 16> T{};
    uint64_t t_ns = 0;
    read_pose(shared, T, t_ns);
    for (int i = 0; i < 16; ++i) {
      pkt.T[i] = T[i];
    }
    pkt.t_ns = t_ns;
    ::sendto(fd, &pkt, sizeof(pkt), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    std::this_thread::sleep_for(period);
  }
}


struct Config {
  std::string robot_ip;
  std::string cmd_bind_ip = "0.0.0.0";
  int cmd_port = 5005;
  std::string pose_target_ip = "10.0.0.2";
  int pose_port = 5006;
  int pose_hz = 200;
  int cmd_timeout_ms = 200;
};

std::string trim_copy(const std::string& s) {
  size_t start = 0;
  while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
    ++start;
  }
  size_t end = s.size();
  while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(start, end - start);
}

bool load_config(const std::string& path, Config& cfg) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    line = trim_copy(line);
    if (line.empty()) {
      continue;
    }
    auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    std::string key = trim_copy(line.substr(0, eq));
    std::string val = trim_copy(line.substr(eq + 1));
    if (key == "robot_ip") {
      cfg.robot_ip = val;
    } else if (key == "cmd_bind_ip") {
      cfg.cmd_bind_ip = val;
    } else if (key == "cmd_port") {
      cfg.cmd_port = std::stoi(val);
    } else if (key == "pose_target_ip") {
      cfg.pose_target_ip = val;
    } else if (key == "pose_port") {
      cfg.pose_port = std::stoi(val);
    } else if (key == "pose_hz") {
      cfg.pose_hz = std::stoi(val);
    } else if (key == "cmd_timeout_ms") {
      cfg.cmd_timeout_ms = std::stoi(val);
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  int argi = 1;

  if (argc >= 3 && std::string(argv[1]) == "--config") {
    if (!load_config(argv[2], cfg)) {
      std::cerr << "Failed to load config: " << argv[2] << "\n";
      return 1;
    }
    argi = 3;
  } else {
    if (!load_config("rt_fci_udp.conf", cfg)) {
      load_config("../rt_fci_udp.conf", cfg);
    }
  }

  if (argi >= argc && cfg.robot_ip.empty()) {
    std::cerr << "Usage: rt_fci_udp [--config path] <robot_ip> [cmd_bind_ip] [cmd_port] [pose_target_ip] [pose_port] [pose_hz] [cmd_timeout_ms]\n";
    return 1;
  }

  if (argi < argc) cfg.robot_ip = argv[argi++];
  if (argi < argc) cfg.cmd_bind_ip = argv[argi++];
  if (argi < argc) cfg.cmd_port = std::stoi(argv[argi++]);
  if (argi < argc) cfg.pose_target_ip = argv[argi++];
  if (argi < argc) cfg.pose_port = std::stoi(argv[argi++]);
  if (argi < argc) cfg.pose_hz = std::stoi(argv[argi++]);
  if (argi < argc) cfg.cmd_timeout_ms = std::stoi(argv[argi++]);


  CmdShared cmd_shared;
  PoseShared pose_shared;

  std::thread rx_thread(cmd_rx_thread, std::ref(cmd_shared), cfg.cmd_bind_ip, cfg.cmd_port);
  std::thread tx_thread(pose_tx_thread, std::cref(pose_shared), cfg.pose_target_ip, cfg.pose_port, cfg.pose_hz);

  try {
    franka::Robot robot(cfg.robot_ip);

    robot.control([&](const franka::RobotState& state, franka::Duration /*period*/) {
      std::array<double, 6> v{};
      uint64_t last_rx_ns = 0;
      read_cmd(cmd_shared, v, last_rx_ns);
      const uint64_t now = now_ns();
      if (last_rx_ns == 0 || now - last_rx_ns > static_cast<uint64_t>(cfg.cmd_timeout_ms) * 1000000ULL) {
        v = {0, 0, 0, 0, 0, 0};
      }

      std::array<double, 16> T{};
      for (int i = 0; i < 16; ++i) {
        T[i] = state.O_T_EE[i];
      }
      write_pose(pose_shared, T, now);

      return franka::CartesianVelocities(v);
    });
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << "\n";
    return 1;
  }

  rx_thread.join();
  tx_thread.join();
  return 0;
}
