#ifndef MACHINERY_HARDWARE_INTERFACE__TOOL_HPP_
#define MACHINERY_HARDWARE_INTERFACE__TOOL_HPP_

#include <libserial/SerialPort.h>

/**
 * @brief 将整数频率转换为 BaudRate 枚举
 * @param speed 整数波特率值 (如 115200)
 * @return 对应的 BaudRate 枚举值
 */
LibSerial::BaudRate intToBaudRate(int speed) {
    switch (speed) {
    case 50:      return LibSerial::BaudRate::BAUD_50;
    case 75:      return LibSerial::BaudRate::BAUD_75;
    case 110:     return LibSerial::BaudRate::BAUD_110;
    case 134:     return LibSerial::BaudRate::BAUD_134;
    case 150:     return LibSerial::BaudRate::BAUD_150;
    case 200:     return LibSerial::BaudRate::BAUD_200;
    case 300:     return LibSerial::BaudRate::BAUD_300;
    case 600:     return LibSerial::BaudRate::BAUD_600;
    case 1200:    return LibSerial::BaudRate::BAUD_1200;
    case 1800:    return LibSerial::BaudRate::BAUD_1800;
    case 2400:    return LibSerial::BaudRate::BAUD_2400;
    case 4800:    return LibSerial::BaudRate::BAUD_4800;
    case 9600:    return LibSerial::BaudRate::BAUD_9600;
    case 19200:   return LibSerial::BaudRate::BAUD_19200;
    case 38400:   return LibSerial::BaudRate::BAUD_38400;
    case 57600:   return LibSerial::BaudRate::BAUD_57600;
    case 115200:  return LibSerial::BaudRate::BAUD_115200;
    case 230400:  return LibSerial::BaudRate::BAUD_230400;

#ifdef __linux__
    case 460800:  return LibSerial::BaudRate::BAUD_460800;
    case 500000:  return LibSerial::BaudRate::BAUD_500000;
    case 576000:  return LibSerial::BaudRate::BAUD_576000;
    case 921600:  return LibSerial::BaudRate::BAUD_921600;
    case 1000000: return LibSerial::BaudRate::BAUD_1000000;
    case 1152000: return LibSerial::BaudRate::BAUD_1152000;
    case 1500000: return LibSerial::BaudRate::BAUD_1500000;

#if defined(B2000000) && __MAX_BAUD > B2000000
    case 2000000: return LibSerial::BaudRate::BAUD_2000000;
    case 2500000: return LibSerial::BaudRate::BAUD_2500000;
    case 3000000: return LibSerial::BaudRate::BAUD_3000000;
    case 3500000: return LibSerial::BaudRate::BAUD_3500000;
    case 4000000: return LibSerial::BaudRate::BAUD_4000000;
#endif
#endif
    default:
        throw std::invalid_argument("不支持该数值的波特率！波特率下限为50,上限为4000000");
    }
}

#endif  // MACHINERY_HARDWARE_INTERFACE__TOOL_HPP_