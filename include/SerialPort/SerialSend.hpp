#pragma once

#include <cstring>
#include <mutex>
#include <array>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

namespace SimulinkBlock
{
/**
     * @brief Класс для отправки данных через последовательный порт
     * @tparam T Тип данных для отправки
     */
template<typename T>
class SerialSend
{
public:
    /**
         * @brief Конструктор для SerialSend
         * @param port_name Имя последовательного порта (например, "/dev/ttyUSB0" или "COM3")
         * @param baud_rate Скорость передачи данных (по умолчанию 9600)
         * @param parity Контроль четности (по умолчанию без контроля)
         * @param character_size Размер символа (по умолчанию 8 бит)
         * @param flow_control Управление потоком (по умолчанию без управления)
         * @param stop_bits Стоповые биты (по умолчанию один)
         */
    SerialSend(const std::string &port_name,
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
         * @brief Отправить данные
         * @param data Данные для отправки
         */
    void send(const T &data)
    {
        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);

        std::array<char, sizeof(T)> buffer;
        std::memcpy(&buffer, &data, sizeof(T));
        serial_port.write_some(boost::asio::buffer(buffer));
    }

    /**
         * @brief Получить список доступных настроек
         * @return Вектор строк с именами настроек
         */
    std::vector<std::string> getSettingsList() const
    {
        return {"port_name", "baud_rate", "parity", "character_size", "flow_control", "stop_bits"};
    }

    /**
         * @brief Установить настройки из карты параметров
         * @param settings Карта параметров (ключ - имя настройки, значение - значение настройки)
         */
    void setSettings(const std::unordered_map<std::string, std::string> &settings)
    {
        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);

        for (const auto &[key, value] : settings) {
            if (key == "baud_rate") {
                try {
                    unsigned int rate = std::stoul(value);
                    setBaudRate(rate);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "parity") {
                setParity(value);
            } else if (key == "character_size") {
                try {
                    unsigned int size = std::stoul(value);
                    setCharacterSize(size);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "flow_control") {
                setFlowControl(value);
            } else if (key == "stop_bits") {
                setStopBits(value);
            }
            // port_name игнорируется, так как порт уже открыт
        }
    }

private:
    /**
         * @brief Установить скорость передачи данных
         * @param rate Скорость передачи данных
         */
    void setBaudRate(unsigned int rate)
    {
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(rate));
    }

    /**
         * @brief Установить контроль четности
         * @param parity_str Строка с типом контроля четности ("none", "odd", "even")
         */
    void setParity(const std::string &parity_str)
    {
        if (parity_str == "none") {
            serial_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        } else if (parity_str == "odd") {
            serial_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
        } else if (parity_str == "even") {
            serial_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
        }
    }

    /**
         * @brief Установить размер символа
         * @param size Размер символа в битах
         */
    void setCharacterSize(unsigned int size)
    {
        serial_port.set_option(boost::asio::serial_port_base::character_size(size));
    }

    /**
         * @brief Установить управление потоком
         * @param flow_str Строка с типом управления потоком ("none", "software", "hardware")
         */
    void setFlowControl(const std::string &flow_str)
    {
        if (flow_str == "none") {
            serial_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        } else if (flow_str == "software") {
            serial_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software));
        } else if (flow_str == "hardware") {
            serial_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware));
        }
    }

    /**
         * @brief Установить стоповые биты
         * @param stop_str Строка с типом стоповых битов ("one", "onepointfive", "two")
         */
    void setStopBits(const std::string &stop_str)
    {
        if (stop_str == "one") {
            serial_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        } else if (stop_str == "onepointfive") {
            serial_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::onepointfive));
        } else if (stop_str == "two") {
            serial_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
        }
    }

private:
    boost::asio::io_context io_context; //!< Служба ввода-вывода для boost::asio
    boost::asio::serial_port serial_port; //!< Последовательный порт для отправки данных
};
}

