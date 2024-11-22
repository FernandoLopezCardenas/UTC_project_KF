#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense> // Biblioteca para operaciones de álgebra lineal
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

// Función para leer datos desde un archivo CSV
MatrixXd readCSV(const string& filename, int rows, int cols) {
    ifstream file(filename); // Abre el archivo

    if (!file.is_open()) { // Verifica si se abrió correctamente
        cerr << "Error: No se pudo abrir el archivo " << filename << endl;
        cerr << "¿Está en la ruta correcta? Intenta usar una ruta absoluta." << endl;
        return MatrixXd::Zero(rows, cols); // Devuelve matriz vacía
    }

    cout << "Archivo abierto exitosamente: " << filename << endl;

    MatrixXd data(rows, cols);
    data.setZero();

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (!(file >> data(i, j))) {
                cerr << "Error leyendo el archivo en la posición (" << i << ", " << j << ")" << endl;
                break;
            }
            if (file.peek() == ',' || file.peek() == '\n') file.ignore();
        }
    }
    return data;
}



int main() 
{
    // Leer datos desde un archivo CSV (reemplazar con el archivo correcto)
    //~/Documents/KALMAN_FILTER/UTC_project_KF/KFC/KFC/KFCODE/src/mainfolder/
    MatrixXd data = readCSV("test.csv", 2, 4); // Ajustar tamaño según el archivo
//    VectorXd ber_mea_x = data.col(0); // Columna X de las mediciones
//    VectorXd ber_mea_y = data.col(1); // Columna Y de las mediciones
//   VectorXd ber_mea_theta = data.col(2); // Orientación (theta)
//    VectorXd ber_mea_throttle = data.col(3); // Velocidad
//    VectorXd ber_mea_delta = data.col(4); // Ángulo de dirección
 //   // Filtrar valores NaN en las mediciones
 //   ber_mea_x = ber_mea_x.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
 //   ber_mea_y = ber_mea_y.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
 //   ber_mea_theta = ber_mea_theta.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
 //   ber_mea_throttle = ber_mea_throttle.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
 //   ber_mea_delta = ber_mea_delta.unaryExpr([](double v) { return isnan(v) ? 0 : v; });

 //   // Generar ruido gaussiano y añadirlo a los datos
 //   int n = ber_mea_x.size(); // Número de datos
 //   MatrixXd ruido_gaussiano = generateGaussianNoise(3, n, 0, 0.1); // Ruido con media 0 y desviación 0.1

 //   // Inicialización del sistema
 //   MatrixXd X = MatrixXd::Zero(3, n); // Estados iniciales
 //   X.row(0) = ber_mea_x.transpose(); // Posición X
 //   X.row(1) = ber_mea_y.transpose(); // Posición Y
 //   X.row(2) = ber_mea_theta.transpose(); // Orientación
 //   X += ruido_gaussiano; // Añadir ruido a los estados

 //   MatrixXd Xk = MatrixXd::Zero(3, n + 1); // Trayectoria estimada
 //   Xk.col(0) << ber_mea_x(0), ber_mea_y(0), ber_mea_theta(0); // Estado inicial
 //   MatrixXd Pk = MatrixXd::Identity(3, 3) * 10; // Covarianza inicial

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
