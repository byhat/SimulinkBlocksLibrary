#pragma once

#include <array>
#include <algorithm>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <sstream>

namespace SimulinkBlock
{
/**
 * @brief Класс для работы с двумерной таблицей поиска (Билинейная интерполяция)
 *
 * @tparam T Тип данных массива и возвращаемого значения
 * @tparam X_size Размер массива по оси X
 * @tparam Y_size Размер массива по оси Y
 */
template <typename T, std::size_t X_size, std::size_t Y_size>
class LookupTable2D
{
private:
    std::shared_mutex mtx;  //!< Мьютекс для блокировки одновременного доступа к переменным класса
    T output = T(0); //!< Значение, интерполированное/экстраполированное из таблицы

    // Ось X и Y должны быть отсортированы по возрастанию для корректной работы std::upper_bound!
    std::array<T, X_size> x; //!< Точки оси X (столбцы)
    std::array<T, Y_size> y; //!< Точки оси Y (строки)

    // Z представлена как Z[Y][X] - строка (Y), затем столбец (X)
    std::array<std::array<T, X_size>, Y_size> z;

public:
    /**
     * @brief Конструктор класса LookupTable2D.
     * @param x_arr Массив точек оси X.
     * @param y_arr Массив точек оси Y.
     * @param z_arr Двумерный массив значений Z[Y][X].
     */
    LookupTable2D(const std::array<T, X_size>& x_arr,
                  const std::array<T, Y_size>& y_arr,
                  const std::array<std::array<T, X_size>, Y_size>& z_arr)
        : x{x_arr}, y{y_arr}, z{z_arr}
    {
    }

    /**
     * @brief Интерполяция/Экстраполяция значения на основе двух входных значений.
     * @param inputX Входное значение по оси X.
     * @param inputY Входное значение по оси Y.
     */
    void interpolate(const T& inputX, const T& inputY)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);

        // 1. Поиск интервала по оси X
        size_t ix1 = 0;
        auto it_x = std::upper_bound(x.begin(), x.end(), inputX);
        if (it_x == x.begin()) ix1 = 0;               // Экстраполяция влево
        else if (it_x == x.end()) ix1 = X_size - 2;   // Экстраполяция вправо
        else ix1 = std::distance(x.begin(), it_x) - 1; // Интерполяция

        size_t ix2 = std::min(ix1 + 1, X_size - 1);

        // 2. Поиск интервала по оси Y
        size_t iy1 = 0;
        auto it_y = std::upper_bound(y.begin(), y.end(), inputY);
        if (it_y == y.begin()) iy1 = 0;               // Экстраполяция вниз
        else if (it_y == y.end()) iy1 = Y_size - 2;   // Экстраполяция вверх
        else iy1 = std::distance(y.begin(), it_y) - 1; // Интерполяция

        size_t iy2 = std::min(iy1 + 1, Y_size - 1);

        // 3. Получение координат углов квадрата
        T x1 = x[ix1], x2 = x[ix2];
        T y1 = y[iy1], y2 = y[iy2];

        // 4. Получение значений Z в углах квадрата
        T q11 = z[iy1][ix1];
        T q12 = z[iy1][ix2];
        T q21 = z[iy2][ix1];
        T q22 = z[iy2][ix2];

        // 5. Вычисление долей смещения (tx, ty)
        // Защита от деления на ноль, если соседние точки по осям сливаются
        T tx = (x2 == x1) ? T(0) : (inputX - x1) / (x2 - x1);
        T ty = (y2 == y1) ? T(0) : (inputY - y1) / (y2 - y1);

        // 6. Билинейная интерполяция
        // Интерполяция по X для нижней (Y1) и верхней (Y2) граней
        T z_y1 = q11 + tx * (q12 - q11);
        T z_y2 = q21 + tx * (q22 - q21);

        // Интерполяция по Y между полученными значениями
        output = z_y1 + ty * (z_y2 - z_y1);
    }

    /**
     * @brief Обратная интерполяция: по X и Z находит Y
     * @param inputX Входное значение по оси X (например, частота)
     * @param inputZ Входное значение Z (например, мощность)
     */
    void interpolateReverseY(const T& inputX, const T& inputZ)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);

        // 1. Находим интервал по оси X (стандартно)
        size_t ix1 = 0;
        auto it_x = std::upper_bound(x.begin(), x.end(), inputX);
        if (it_x == x.begin()) ix1 = 0;
        else if (it_x == x.end()) ix1 = X_size - 2;
        else ix1 = std::distance(x.begin(), it_x) - 1;

        size_t ix2 = std::min(ix1 + 1, X_size - 1);
        T x1 = x[ix1], x2 = x[ix2];

        // Лямбда-функция для поиска Y по срезу Z при фиксированном X
        auto findYForZSlice = [&](size_t ix) -> T {
            // Собираем столбец Z для текущей X
            std::array<T, Y_size> z_slice;
            for (size_t i = 0; i < Y_size; ++i) {
                z_slice[i] = z[i][ix];
            }

            // Ищем позицию inputZ в УБЫВАЮЩЕМ массиве z_slice.
            // Для убывающего массива используем компаратор std::greater<T>()
            auto it_z = std::upper_bound(z_slice.begin(), z_slice.end(), inputZ, std::greater<T>());

            size_t iy1 = 0;
            if (it_z == z_slice.begin()) iy1 = 0;               // Экстраполяция
            else if (it_z == z_slice.end()) iy1 = Y_size - 2;   // Экстраполяция
            else iy1 = std::distance(z_slice.begin(), it_z) - 1; // Интерполяция

            size_t iy2 = std::min(iy1 + 1, Y_size - 1);

            T z1 = z_slice[iy1];
            T z2 = z_slice[iy2];
            T y1 = y[iy1];
            T y2 = y[iy2];

            // Доля смещения по оси Z
            T tz = (z1 == z2) ? T(0) : (inputZ - z1) / (z2 - z1);

            // Линейная интерполяция, чтобы получить Y
            return y1 + tz * (y2 - y1);
        };

        // 2. Находим Y для левой границы X
        T y_at_x1 = findYForZSlice(ix1);
        // 3. Находим Y для правой границы X
        T y_at_x2 = findYForZSlice(ix2);

        // 4. Финальная интерполяция между полученными Y по оси X
        T tx = (x2 == x1) ? T(0) : (inputX - x1) / (x2 - x1);

        output = y_at_x1 + tx * (y_at_x2 - y_at_x1);
    }

    /**
     * @brief Получить текущее выходное значение
     * @return Ссылка на текущее значение, экстраполированное из таблицы
     */
    const T& getOutput()
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        return output;
    }

    /**
     * @brief Получить список доступных настроек
     * @return Вектор строк с именами настроек
     */
    std::vector<std::string> getSettingsList() const
    {
        return {"x_array", "y_array", "z_array"};
    }

    /**
     * @brief Установить настройки из карты параметров
     * @param settings Карта параметров (ключ - имя настройки, значение - значение настройки)
     */
    void setSettings(const std::unordered_map<std::string, std::string> &settings)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);

        for (const auto &[key, value] : settings) {
            try {
                if (key == "x_array") {
                    x = parseArray<X_size>(value);
                } else if (key == "y_array") {
                    y = parseArray<Y_size>(value);
                } else if (key == "z_array") {
                    z = parse2DArray(value);
                }
            } catch (...) {
                // Игнорируем ошибки преобразования
            }
        }
    }

    /**
      * @brief Обнулить текущий выход блока
      */
    void reset()
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        output = T(0);
    }

private:
    /**
     * @brief Разобрать строку с разделителями-запятыми в 1D массив
     * @tparam N Размер массива
     * @param str Строка с разделителями-запятыми
     * @return Массив значений типа T
     */
    template <std::size_t N>
    std::array<T, N> parseArray(const std::string &str)
    {
        std::array<T, N> result;
        std::stringstream ss(str);
        std::string token;
        size_t index = 0;

        while (std::getline(ss, token, ',') && index < N) {
            try {
                result[index] = static_cast<T>(std::stod(token));
                ++index;
            } catch (...) {
                // Игнорируем ошибки преобразования отдельных значений
            }
        }
        return result;
    }

    /**
     * @brief Разобрать строку с разделителями-запятыми в 2D массив (Z[Y][X])
     * @param str Строка с разделителями-запятыми (значения идут строка за строкой)
     * @return Двумерный массив значений типа T
     */
    std::array<std::array<T, X_size>, Y_size> parse2DArray(const std::string &str)
    {
        std::array<std::array<T, X_size>, Y_size> result;
        std::stringstream ss(str);
        std::string token;
        size_t row = 0, col = 0;

        while (std::getline(ss, token, ',')) {
            try {
                result[row][col] = static_cast<T>(std::stod(token));
                col++;
                if (col == X_size) {
                    col = 0;
                    row++;
                    if (row == Y_size) break;
                }
            } catch (...) {
                // Игнорируем ошибки
            }
        }
        return result;
    }
};
}
