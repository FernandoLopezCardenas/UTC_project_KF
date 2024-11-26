//
// Created by ateveraz on 23/11/24.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

// Función para leer datos CSV
Matrix<double, Dynamic, Dynamic> csv2eigenMatrix(const string& filename)
{
    // Abrir archivo
    fstream dataset;

    dataset.open(filename, ios::in);

    // Verificar si el archivo se abrió correctamente
    if (!dataset.is_open())
    {
        cout << "Error al abrir el archivo csv" << endl;
        return Matrix<double, Dynamic, Dynamic>(0, 0);
    }

    // Leer datos del archivo
    string line, word;
    vector<string> row;
    vector<vector<string>> data;

    getline(dataset, line);
    while (getline(dataset, line))
    {
        row.clear();
        stringstream s(line);
        while (getline(s, word, ','))
        {
            row.push_back(word);
        }
        data.push_back(row);
    }

    // Convertir datos a matriz Eigen
    int rows = data.size();
    int cols = data[0].size();

    Matrix<double, Dynamic, Dynamic> dataMatrix(rows, cols);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            dataMatrix(i, j) = stod(data[i][j]);
        }
    }

    // Cerrar archivo
    dataset.close();

    return dataMatrix;
}

int main() {
    // Leer archivo mediante la función. 
    Matrix<double, Dynamic, Dynamic> dataset = csv2eigenMatrix("collectedData.csv");

    // Plot de verificación (subset). Ya en la variable dataset están los datos que quieras procesar. 
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            std::cout << dataset(i, j) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}


