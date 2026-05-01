#pragma once

#include "IntegratorBlock.hpp"
#include "DerivativeBlock.hpp"
#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <string>
#include <sstream>


namespace SimulinkBlock
{
/**
 * @brief Класс, реализующий блок ПИД регулятора
 *
 * @tparam T Тип прошлого состояния и входа для блока
 */
template <typename T>
class PID
{
private:
    std::mutex mtx; //!< Мьютекс для блокировки одновременного доступа к переменным класса

    DerivativeBlock<T> derivative; //!< Блок дифференцирования
    IntegratorBlock<T> integrator; //!< Блок интегрирования

    T pidOutput = T{0}; //!< Выход блока ПИД регулятора

    T P = T{0}; //!< Коэффициент усиления для пропорциональной составляющей
    T I = T{0}; //!< Коэффициент усиления для интегрирующей составляющей
    T D = T{0}; //!< Коэффициент усиления для дифференцирующей составляющей

public:
    PID() = default;
    /**
     * @brief Конструктор для инициализации блока ПИД регулятора с заданными пределами по умолчанию
     * @param P Коэффициент усиления для пропорциональной составляющей
     * @param I Коэффициент усиления для интегрирующей составляющей
     * @param D Коэффициент усиления для дифференцирующей составляющей
     */
    PID(const T &p, const T &i, const T &d) : P{p}, I{i}, D{d}
    {}

    /**
     * @brief Конструктор для инициализации блока ПИД регулятора с заданными пределами по умолчанию
     *
     * @param P Коэффициент усиления для пропорциональной составляющей
     * @param I Коэффициент усиления для интегрирующей составляющей
     * @param D Коэффициент усиления для дифференцирующей составляющей
     * @param minI Минимальный предел  интегрирования
     * @param maxI Максимальный предел интегрирования
     * @param min Минимальный предел дифференцирования
     * @param max Максимальный предел дифференцирования
     */
    PID(const T &p, const T &i, const T &d,
        const T& minI, const T& maxI, const T& minD, const T& maxD)
        : P{p}, I{i}, D{d}
    {
        setLimits(minI, maxI, minD, maxD);
    }

    /**
     * @brief Установить коэффициенты для блока ПИД регулятора
     *
     * @param P Коэффициент усиления для пропорциональной составляющей
     * @param I Коэффициент усиления для интегрирующей составляющей
     * @param D Коэффициент усиления для дифференцирующей составляющей
     */
    void setCoeffs(const T &p, const T &i, const T &d)
    {
        std::lock_guard<std::mutex> lock(mtx);
        P = p;
        I = i;
        D = d;
    }

    /**
     * @brief Установить коэффициент П для блока ПИД регулятора
     */
    void setPCoeff(const T &p)
    {
        std::lock_guard<std::mutex> lock(mtx);
        P = p;
    }

    /**
     * @brief Установить коэффициент И для блока ПИД регулятора
     */
    void setICoeff(const T &i)
    {
        std::lock_guard<std::mutex> lock(mtx);
        I = i;
    }

    /**
     * @brief Установить коэффициент Д для блока ПИД регулятора
     */
    void setDCoeff(const T &d)
    {
        std::lock_guard<std::mutex> lock(mtx);
        D = d;
    }

    /**
     * @brief Установить пределы результата дифференцирования
     *
     * @param minI Минимальный предел  интегрирования
     * @param maxI Максимальный предел интегрирования
     * @param minD Минимальный предел  дифференцирования
     * @param maxD Максимальный предел дифференцирования
     */
    void setLimits(const T& minI, const T& maxI, const T& minD, const T& maxD)
    {
        std::lock_guard<std::mutex> lock(mtx);
        integrator.setLimits(minI, maxI);
        derivative.setLimits(minD, maxD);
    }

    /**
     * @brief Установить пределы результата интегрирования
     *
     * @param min Минимальный предел интегрирования
     * @param max Максимальный предел интегрирования
     */
    void setIntegratorLimits(const T& min, const T& max)
    {
        std::lock_guard<std::mutex> lock(mtx);
        integrator.setLimits(min, max);
    }

    /**
     * @brief Установить пределы результата дифференцирования
     *
     * @param min Минимальный предел дифференцирования
     * @param max Максимальный предел дифференцирования
     */
    void setDerivativeLimits(const T& min, const T& max)
    {
        std::lock_guard<std::mutex> lock(mtx);
        derivative.setLimits(min, max);
    }

    /**
     * @brief Выполнить один шаг блока ПИД регулятора
     *
     * @param input Входное значение
     * @param dt Временной шаг
     */
    void step(const T& input, double dt)
    {
        std::lock_guard<std::mutex> lock(mtx);
        integrator.step(input, dt);
        derivative.step(input, dt);
        pidOutput = P * input + integrator.getOutput() * I + derivative.getOutput() * D;
    }

    /**
     * @brief Установить состояние блока дифференцирования
     *
     * @param newPrevInput Новое значение состояния для установки
     */
    void setDerivativeState(const T& newPrevInput)
    {
        std::lock_guard<std::mutex> lock(mtx);
        derivative.setState(newPrevInput);
    }

    /**
     * @brief Установить состояние блока интегрирования
     *
     * @param newState Новое значение состояния для установки
     */
    void setIntegratorState(const T& newState)
    {
        std::lock_guard<std::mutex> lock(mtx);
        integrator.setState(newState);
    }

    /**
     * @brief Получить ссылку на выходные данные блока
     *
     * @return Ссылка на выходные данные
     */
    const T& getOutput()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return pidOutput;
    }

    /**
     * @brief Получить список доступных настроек
     * @return Вектор строк с именами настроек
     */
    std::vector<std::string> getSettingsList() const
    {
        return {"kp", "ki", "kd", "integrator_min", "integrator_max", "derivative_min", "derivative_max"};
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
            if (key == "kp") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    setPCoeff(val);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "ki") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    setICoeff(val);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "kd") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    setDCoeff(val);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "integrator_min") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    T max = integrator.getState(); // Временное значение для получения текущего максимума
                    setIntegratorLimits(val, max);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "integrator_max") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    T min = T{0}; // Временное значение для получения текущего минимума
                    setIntegratorLimits(min, val);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "derivative_min") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    T max = T{10000}; // Значение по умолчанию
                    setDerivativeLimits(val, max);
                } catch (...) {
                    // Игнорируем ошибки преобразования
                }
            } else if (key == "derivative_max") {
                try {
                    T val = static_cast<T>(std::stod(value));
                    T min = static_cast<T>(-10000); // Значение по умолчанию
                    setDerivativeLimits(min, val);
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
        P = T{0};
        I = T{0};
        D = T{0};
        derivative.reset();
        integrator.reset();
        pidOutput = T{0};
    }
};
}
