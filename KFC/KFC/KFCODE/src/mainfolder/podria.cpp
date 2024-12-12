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
MatrixXd readCSV(const string& filename, int rows, int cols)
{
    ifstream file(filename); // Abre el archivo en modo lectura
    MatrixXd data(rows, cols); // Matriz para almacenar los datos
    if (file.is_open()) { // Verifica si el archivo se abrió correctamente
        for (int i = 0; i < rows; ++i) { // Recorre las filas
            for (int j = 0; j < cols; ++j) { // Recorre las columnas
                file >> data(i, j); // Lee el valor y lo almacena en la matriz
                if (file.peek() == ',' || file.peek() == '\n') file.ignore(); // Ignora separadores
            }
        }
    }
    return data; // Devuelve la matriz con los datos leídos
}

// Función para generar ruido gaussiano
MatrixXd generateGaussianNoise(int rows, int cols, double mu, double sigma)
{
    default_random_engine generator(time(0)); // Generador de números aleatorios
    normal_distribution<double> distribution(mu, sigma); // Distribución normal con media mu y desviación sigma
    MatrixXd noise(rows, cols); // Matriz para el ruido
    for (int i = 0; i < rows; ++i) { // Recorre las filas
        for (int j = 0; j < cols; ++j) { // Recorre las columnas
            noise(i, j) = distribution(generator); // Genera un número aleatorio
            noise(i, j) = min(max(noise(i, j), -1.0), 1.0); // Limita el valor a [-1, 1]
        }
    }
    return noise; // Devuelve la matriz de ruido
}

// Implementación del Filtro de Kalman Extendido
void extendedKalmanFilter(MatrixXd& X, const VectorXd& Dk, const VectorXd& delta, double L, MatrixXd& Xk, MatrixXd& Pk) {
    int n = X.cols(); // Número de columnas (datos de entrada)
    MatrixXd Id = MatrixXd::Identity(3, 3); // Matriz identidad 3x3
    MatrixXd Hk = MatrixXd::Identity(3, 3); // Matriz de observación (identidad en este caso)
    MatrixXd Qk, Rk; // Matrices de ruido de proceso y medición

    for (int i = 0; i < n; ++i) {
        // Condicional para ajustar Qk y Rk si el dato es cero
        if (X.col(i).isZero()) {
            Qk = MatrixXd::Identity(3, 3) * 1500; // Ruido de proceso grande
            Rk = MatrixXd::Identity(3, 3) * 1e10; // Ruido de medición grande
        } else {
            Qk = MatrixXd::Identity(3, 3) * 15; // Ruido de proceso bajo
            Rk = MatrixXd::Identity(3, 3) * 0.1 * 0.1; // Ruido de medición bajo
        }

        // Predicción del estado (modelo de movimiento)
        VectorXd Xk_i = Xk.col(i); // Estado actual
        Xk(0, i + 1) = Xk_i(0) + Dk(i) * cos(delta(i)) * cos(Xk_i(2)); // Posición X
        Xk(1, i + 1) = Xk_i(1) + Dk(i) * cos(delta(i)) * sin(Xk_i(2)); // Posición Y
        Xk(2, i + 1) = Xk_i(2) + Dk(i) / L * sin(delta(i)); // Orientación

        // Cálculo de la matriz Jacobiana (modelo linealizado)
        MatrixXd Fk(3, 3);
        Fk << 1, 0, -Dk(i) * cos(delta(i)) * sin(Xk_i(2)),
              0, 1, Dk(i) * cos(delta(i)) * cos(Xk_i(2)),
              0, 0, 1;

        // Predicción de la covarianza
        Pk = Fk * Pk * Fk.transpose() + Qk;

        // Nota: No se incluye la corrección con Zk, pero aquí podría implementarse
    }
}

int main() {
    // Leer datos desde un archivo CSV (reemplazar con el archivo correcto)
    MatrixXd data = readCSV("collectedData.csv", 1000, 5); // Ajustar tamaño según el archivo
    VectorXd ber_mea_x = data.col(0); // Columna X de las mediciones
    VectorXd ber_mea_y = data.col(1); // Columna Y de las mediciones
    VectorXd ber_mea_theta = data.col(2); // Orientación (theta)
    VectorXd ber_mea_throttle = data.col(3); // Velocidad
    VectorXd ber_mea_delta = data.col(4); // Ángulo de dirección

    // Filtrar valores NaN en las mediciones
    ber_mea_x = ber_mea_x.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_y = ber_mea_y.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_theta = ber_mea_theta.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_throttle = ber_mea_throttle.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_delta = ber_mea_delta.unaryExpr([](double v) { return isnan(v) ? 0 : v; });

    // Generar ruido gaussiano y añadirlo a los datos
    int n = ber_mea_x.size(); // Número de datos
    MatrixXd ruido_gaussiano = generateGaussianNoise(3, n, 0, 0.1); // Ruido con media 0 y desviación 0.1

    // Inicialización del sistema
    MatrixXd X = MatrixXd::Zero(3, n); // Estados iniciales
    X.row(0) = ber_mea_x.transpose(); // Posición X
    X.row(1) = ber_mea_y.transpose(); // Posición Y
    X.row(2) = ber_mea_theta.transpose(); // Orientación
    X += ruido_gaussiano; // Añadir ruido a los estados

    MatrixXd Xk = MatrixXd::Zero(3, n + 1); // Trayectoria estimada
    Xk.col(0) << ber_mea_x(0), ber_mea_y(0), ber_mea_theta(0); // Estado inicial
    MatrixXd Pk = MatrixXd::Identity(3, 3) * 10; // Covarianza inicial

    double L = 0.155; // Distancia entre ejes del vehículo
    VectorXd Dk = ber_mea_throttle; // Velocidades medidas
    VectorXd delta = ber_mea_delta; // Ángulos de dirección medidos

    // Aplicar el Filtro de Kalman Extendido
    extendedKalmanFilter(X, Dk, delta, L, Xk, Pk);

    // Mostrar resultados
    cout << "Trayectoria estimada Xk:\n" << Xk << endl;
    cout << "Covarianza final Pk:\n" << Pk << endl;

    return 0;
}
