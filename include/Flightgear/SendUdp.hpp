#pragma once

#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/asio.hpp>

namespace SimulinkBlock
{
/**
     * @brief Класс для отправки данных по протоколу UDP
     * @tparam T Тип данных для отправки
     */
template<typename T>
class SendUdp
{
public:
    /**
         * @brief Конструктор для DataSender
         * @param ip IP-адрес, на который будут отправлены данные
         * @param port Номер порта, на который будут отправлены данные
         */
    SendUdp(const std::string &ip = "127.0.0.1", unsigned short port = 5502) :
        socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)),
        server_endpoint(boost::asio::ip::make_address(ip), port)
    {
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
        socket.send_to(boost::asio::buffer(buffer), server_endpoint);
    }

    /**
         * @brief Получить список доступных настроек
         * @return Вектор строк с именами настроек
         */
    std::vector<std::string> getSettingsList() const
    {
        return {"ip", "port"};
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
            if (key == "ip") {
                setIp(value);
            } else if (key == "port") {
                try {
                    unsigned short port_num = static_cast<unsigned short>(std::stoul(value));
                    setPort(port_num);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            }
        }
    }

private:
    /**
         * @brief Установить IP-адрес назначения
         * @param ip_str Строка с IP-адресом
         */
    void setIp(const std::string &ip_str)
    {
        try {
            auto new_ip = boost::asio::ip::make_address(ip_str);
            server_endpoint = boost::asio::ip::udp::endpoint(new_ip, server_endpoint.port());
        } catch (...) {
            // Игнорируем ошибки парсинга IP-адреса
        }
    }

    /**
         * @brief Установить порт назначения
         * @param port_num Номер порта
         */
    void setPort(unsigned short port_num)
    {
        server_endpoint = boost::asio::ip::udp::endpoint(server_endpoint.address(), port_num);
    }

    boost::asio::io_context io_context; //!< Служба ввода-вывода для boost::asio
    boost::asio::ip::udp::socket socket; //!< UDP-сокет для отправки данных
    boost::asio::ip::udp::endpoint server_endpoint; //!< Конечная точка сервера для отправки данных
};
}
