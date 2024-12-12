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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Función para generar una matriz de ruido gaussiano
MatrixXd generarRuidoGaussiano(int filas, int columnas, double media, double desviacionEstandar) {
    // Inicializar generador de números aleatorios
    std::random_device rd;
    std::mt19937 generador(rd());
    std::normal_distribution<double> distribucion(media, desviacionEstandar);

    // Crear la matriz de tamaño filas x columnas
    MatrixXd matrizRuido(filas, columnas);

    // Rellenar la matriz con valores aleatorios gausianos
    for (int i = 0; i < filas; ++i) {
        for (int j = 0; j < columnas; ++j) {
            matrizRuido(i, j) = distribucion(generador);
        }
    }

    return matrizRuido;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Función para leer datos CSV
Matrix<double, Dynamic, Dynamic> csv2eigenMatrix(const string& filename)
{
    // Abrir archivo
    fstream dataset;

    dataset.open(filename, ios::in);

    // Verificar si el archivo se abrió correctamente
    if (!dataset.is_open())
    {
        cout << "Error al abrir el archivo csv: "<< filename << endl;
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
//    for (int i = 0; i < dataset.rows(); i++)
//    {
//        for (int j = 0; j < dataset.cols(); j++)
//        {
//            std::cout << dataset(i, j) << " ";
//        }
//        std::cout << std::endl;
//    } // Ajustar tamaño según el archivo
    VectorXd ber_mea_x = dataset.col(0); // Columna X de las mediciones
    VectorXd ber_mea_y = dataset.col(1); // Columna Y de las mediciones
    VectorXd ber_mea_theta = dataset.col(2); // Orientación (theta)
//    VectorXd ber_mea_throttle = dataset.col(3); // Velocidad
//    VectorXd ber_mea_delta = dataset.col(4); // Ángulo de dirección

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Generar ruido gaussiano y añadirlo a los datos
    int n = ber_mea_x.size(); // Número de datos
    // Generar la matriz de ruido gaussiano
    MatrixXd ruido_gaussiano = generarRuidoGaussiano(3, n, 0.0, 0.1);

    // Imprimir la matriz
//    std::cout << "Matriz de ruido gaussiano:\n" << ruido_gaussiano << std::endl;
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Inicialización del sistema
    MatrixXd X = MatrixXd::Zero(3, n); // Estados iniciales
    X.row(0) = ber_mea_x.transpose(); // Posición X
    X.row(1) = ber_mea_y.transpose(); // Posición Y
    X.row(2) = ber_mea_theta.transpose(); // Orientación
    //std::cout << X.col(45);
    X += ruido_gaussiano; // Añadir ruido a los estados
    //std::cout << X.col(45);
    MatrixXd Xk = MatrixXd::Zero(3, n + 1); // Trayectoria estimada
    Xk.col(0) << ber_mea_x(0), ber_mea_y(0), ber_mea_theta(0); // Estado inicial
    MatrixXd Pk = MatrixXd::Identity(3, 3) * 10; // Covarianza inicial
    std::cout << Xk;
 //   double L = 0.155; // Distancia entre ejes del vehículo
 //   VectorXd Dk = ber_mea_throttle; // Velocidades medidas
 //   VectorXd delta = ber_mea_delta; // Ángulos de dirección medidos

 //   // Aplicar el Filtro de Kalman Extendido
 //   extendedKalmanFilter(X, Dk, delta, L, Xk, Pk);

 //   // Mostrar resultados
 //   cout << "Trayectoria estimada Xk:\n" << Xk << endl;
 //   cout << "Covarianza final Pk:\n" << Pk << endl;

    return 0;
}
