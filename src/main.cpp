#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <type_traits>

#include "main.hpp"

void print_help(const char* const self) {
  std::cout << "Usage: " << self << " [options]\n"
    "Options:\n"
    "  -h         Show this help message\n"
    "  -f FILE    Read input from FILE instead of standard input\n"
    "  -g         Enable debug output\n"
    "  -p         Preempt targets if they are not reached when new command arrives\n"
    "  -q PERIOD  Encoders update period\n"
    ;
}

int main(const int argc, const char* const argv[]) {
  if (argc < 1) {
    std::cerr << "Invalid arguments\n";
    return 1;
  }
  std::optional<const char*> input_file;
  Constraints constraints;
  bool preempt = false;
  double encoder_period = 0.05;
  int i = 1;
  bool debug = false;
  auto get_next_arg = [&argc, &argv, &i]() -> std::optional<const char*> {
    ++i;
    if (i >= argc) {
      return std::nullopt;
    }
    return argv[i];
  };
  for (; i < argc; ++i) {
    if (std::strlen(argv[i]) != 2 || argv[i][0] != '-') {
      std::cerr << "Argument " << argv[i][0] << " is not valid write -h for help\n";
      return 1;
    }
    switch (argv[i][1]) {
      default: {
        std::cerr << "Argument " << argv[i][0] << " is not valid write -h for help\n";
        return 1;
      }
      case 'h': {
        print_help(argv[0]);
        return 0;
      }
      case 'f': {
        auto arg = get_next_arg();
        if (!arg.has_value()) {
          std::cerr << "Option -f requires a file name\n";
          return 1;
        }
        if (!std::filesystem::exists(*arg)) {
          std::cerr << "File " << *arg << " does not exist\n";
          return 1;
        }
        input_file = *arg;
        break;
      }
      case 't': {
        auto arg = get_next_arg();
        if (!arg.has_value()) {
          std::cerr << "Option -t requires a number\n";
          return 1;
        }
        try {
          auto v = std::stoul(*arg);
          if (v > 4095 || v < 0) {
            std::cerr << "Option -t requires a number in range [0, 4095]\n";
            return 1;
          }
          constraints.vertical_max = static_cast<uint16_t>(v);
        } catch (const std::exception&) {
          std::cerr << "Option -t requires a valid number\n";
          return 1;
        }
        break;
      }
      case 'd': {
        auto arg = get_next_arg();
        if (!arg.has_value()) {
          std::cerr << "Option -d requires a number\n";
          return 1;
        }
        try {
          auto v = std::stoul(*arg);
          if (v > 4095 || v < 0) {
            std::cerr << "Option -d requires a number in range [0, 4095]\n";
            return 1;
          }
          constraints.vertical_min = static_cast<uint16_t>(v);
        } catch (const std::exception&) {
          std::cerr << "Option -d requires a valid number\n";
          return 1;
        }
        break;
      }
      case 'l': {
        auto arg = get_next_arg();
        if (!arg.has_value()) {
          std::cerr << "Option -l requires a number\n";
          return 1;
        }
        try {
          auto v = std::stoul(*arg);
          if (v > 4095 || v < 0) {
            std::cerr << "Option -l requires a number in range [0, 4095]\n";
            return 1;
          }
          constraints.horizontal_min = static_cast<uint16_t>(v);
        } catch (const std::exception&) {
          std::cerr << "Option -l requires a valid number\n";
          return 1;
        }
        break;
      }
      case 'r': {
        auto arg = get_next_arg();
        if (!arg.has_value()) {
          std::cerr << "Option -r requires a number\n";
          return 1;
        }
        try {
          auto v = std::stoul(*arg);
          if (v > 4095 || v < 0) {
            std::cerr << "Option -r requires a number in range [0, 4095]\n";
            return 1;
          }
          constraints.horizontal_max = static_cast<uint16_t>(v);
        } catch (const std::exception&) {
          std::cerr << "Option -r requires a valid number\n";
          return 1;
        }
    }
    case 'p': {
      preempt = true;
      break;
    }
    case 'q': {
        auto arg = get_next_arg();
        if (!arg.has_value()) {
          std::cerr << "Option -q requires a number\n";
          return 1;
        }
        try {
          encoder_period = std::stod(*arg);
          if (encoder_period <= 0.0) {
            std::cerr << "Option -q requires a positive number\n";
            return 1;
          }
        } catch (const std::exception&) {
          std::cerr << "Option -q requires a valid number\n";
          return 1;
        }
        break;
      }
      case 'g': {
        debug = true;
        break;
      }
    }
  }
  if (input_file.has_value()) {
    std::ifstream file_stream(*input_file);
    if (!file_stream.is_open()) {
      std::cerr << "Failed to open file " << *input_file << "\n";
      return 1;
    }
    run_tests(file_stream, encoder_period, debug, constraints, preempt);
  } else {
    std::cout << "Reading from standard input, press Ctrl+D (or Ctrl+Z on Windows) to end input\n";
    run_tests(std::cin, encoder_period, debug, constraints, preempt);
  }
  return 0;
}