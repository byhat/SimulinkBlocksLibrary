#pragma once

#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <string>

namespace SimulinkBlock
{
/**
 * @brief Класс, реализующий логику работы блока насыщения
 *
 * @tparam T Тип входа и выхода для блока насыщения
 */
template <typename T>
class SaturationBlock
{
private:
    std::mutex mtx;  //!< Мьютекс для блокировки одновременного доступа к переменным класса
    T output = T(0); //!< Выход блока насыщения
    T minLimit;      //!< Минимальный предел насыщения
    T maxLimit;      //!< Максимальный предел насыщения

public:
    /**
     * @brief Конструктор для инициализации блока насыщения с заданными пределами по умолчанию
     *
     * @param input Входной сигнал
     * @param minLimit Нижняя граница ограничения значения
     * @param maxLimit Верхняя граница ограничения значения
     */
    SaturationBlock(const T& min = std::numeric_limits<T>::min(),
                    const T& max = std::numeric_limits<T>::max())
        : minLimit{min},
          maxLimit{max}
    {
    }

    /**
     * @brief Выполнить один шаг расчёта блока насыщения
     *
     * @param input Входное значение для блока насыщения
     */
    void step(const T& input)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if ( input > maxLimit)
            output = maxLimit;
        else if ( input < minLimit)
            output = minLimit;
        else
            output = input;
    }

    /**
     * @brief Установить пределы результата насыщения
     *
     * @param min Минимальный предел насыщения
     * @param max Максимальный предел насыщения
     */
    void setLimits(const T& min, const T& max)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if(min > max)
        {
            std::swap(maxLimit, minLimit);
            throw std::invalid_argument("Min value should not be greater than max value");
            return;
        }
        minLimit = min;
        maxLimit = max;
    }

    /**
     * @brief Ссылка на текущее состояние блока насыщения
     *
     * @return Указатель на сигнал с учётом ограничений
     */
    const T& getOutput()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return output;
    }

    /**
     * @brief Получить список доступных настроек
     * @return Вектор строк с именами настроек
     */
    std::vector<std::string> getSettingsList() const
    {
        return {"min_limit", "max_limit"};
    }

    /**
     * @brief Установить настройки из карты параметров
     * @param settings Карта параметров (ключ - имя настройки, значение - значение настройки)
     */
    void setSettings(const std::unordered_map<std::string, std::string> &settings)
    {
        static std::mutex setMtx;
        std::lock_guard<std::mutex> lock(setMtx);

        for (const auto &[key, value] : settings) {
            if (key == "min_limit") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    setLimits(val, maxLimit);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "max_limit") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    setLimits(minLimit, val);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            }
        }
    }

    /**
      * @brief Обнулить текущий выход блока
      */
    void reset()
    {
        std::lock_guard<std::mutex> lock(mtx);
        output = T(0);
    }
};
}
