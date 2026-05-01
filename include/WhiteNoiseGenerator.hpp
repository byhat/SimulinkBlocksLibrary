#pragma once

#include <mutex>
#include <random>
#include <unordered_map>
#include <vector>
#include <string>


namespace SimulinkBlock
{
/**
 * @brief Шаблон класса для генерации белого шума типа T
 */
template <typename T>
class WhiteNoiseGenerator
{
private:
    std::mutex mtx;                           //!< Мьютекс для блокировки одновременного доступа к переменным класса
    std::mt19937 generator;                   //!< Генератор случайных чисел Mersenne Twister
    std::normal_distribution<T> distribution; //!< Нормальное распределение для генерации белого шума
    T output = T(0);                          //!< Переменная для хранения сгенерированного значения белого шума

public:
    /**
     * @brief Конструктор, инициализирующий генератор случайных чисел и нормальное распределение
     */
    WhiteNoiseGenerator(T mean, T stddev)
        : generator(std::random_device()()),
        distribution(mean, stddev)
    {
    }

    /**
     * @brief Функция для генерации белого шума
     */
    void step()
    {
        std::lock_guard<std::mutex> lock(mtx);
        output = distribution(generator);
    }

    /**
     * @brief Получить указатель на выходные данные блока генерации белого шума
     *
     * @return Указатель на выходные данные
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
        return {"mean", "std_dev"};
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
            if (key == "mean") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    distribution = std::normal_distribution<T>(val, distribution.stddev());
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "std_dev") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    distribution = std::normal_distribution<T>(distribution.mean(), val);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            }
        }
    }

    /**
      * @brief Обнулить текущее состояние блока
      */
    void reset()
    {
        std::lock_guard<std::mutex> lock(mtx);
        output = T(0);
    }
};
}
