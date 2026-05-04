#include <iostream>

#include "../../include/SimulinkBlocksLibrary.hpp"


using namespace SimulinkBlock;

int main()
{
    std::array<double, 6> x_arr = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    std::array<double, 2> y_arr = {
        30.0, 20.0
    };

    std::array<std::array<double, 6>, 2> z_arr_correct = {{
        {123, 223, 344, 3256, 574, 457},
        {459, 457, 456, 475, 455, 457},
    }};

    SimulinkBlock::LookupTable2D<double, 6, 2> myTable(x_arr, y_arr, z_arr_correct);

    // myTable.interpolate(4.0, 22.0);
    myTable.interpolateReverseY(4.0, 475.0);

    std::cout << "Результат интерполяции: " << myTable.getOutput() << '\n';
              
    return 0;
}
