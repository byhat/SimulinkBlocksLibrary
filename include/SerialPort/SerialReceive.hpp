#pragma once

#include <cstring>
#include <mutex>
#include <array>
#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

namespace SimulinkBlock
{
/**
     * @brief Класс для приема данных через последовательный порт
     * @tparam T Тип данных для приема
     */
template<typename T>
class SerialReceive
{
public:
    /**
         * @brief Конструктор для SerialReceive
         * @param port_name Имя последовательного порта (например, "/dev/ttyUSB0" или "COM3")
         * @param baud_rate Скорость передачи данных (по умолчанию 9600)
         * @param parity Контроль четности (по умолчанию без контроля)
         * @param character_size Размер символа (по умолчанию 8 бит)
         * @param flow_control Управление потоком (по умолчанию без управления)
         * @param stop_bits Стоповые биты (по умолчанию один)
         */
    SerialReceive(const std::string &port_name,
                  unsigned int baud_rate = 9600,
                  boost::asio::serial_port_base::parity parity = boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
                  boost::asio::serial_port_base::character_size character_size = boost::asio::serial_port_base::character_size(8),
                  boost::asio::serial_port_base::flow_control flow_control = boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),
                  boost::asio::serial_port_base::stop_bits stop_bits = boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)) :
        serial_port(io_context, port_name)
    {
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port.set_option(parity);
        serial_port.set_option(character_size);
        serial_port.set_option(flow_control);
        serial_port.set_option(stop_bits);
    }

    /**
         * @brief Принять данные
         * @param data Ссылка на переменную для сохранения принятых данных
         * @return true если прием успешен, false при ошибке
         */
    bool receive(T &data)
    {
        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);

        try {
            serial_port.read_some(boost::asio::buffer(buffer));
            std::memcpy(&data, buffer.data(), sizeof(T));
            return true;
        } catch (...) {
            return false;
        }
    }

private:
    boost::asio::io_context io_context; //!< Служба ввода-вывода для boost::asio
    boost::asio::serial_port serial_port; //!< Последовательный порт для приема данных
    std::array<char, sizeof(T)> buffer; //!< Буфер для хранения принятых данных
};
}
