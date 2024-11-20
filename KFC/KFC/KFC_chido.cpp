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
MatrixXd readCSV(const string& filename, int rows, int cols) {
    ifstream file(filename);
    MatrixXd data(rows, cols);
    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file >> data(i, j);
                if (file.peek() == ',' || file.peek() == '\n') file.ignore();
            }
        }
    }
    return data;
}

// Función para generar ruido gaussiano
MatrixXd generateGaussianNoise(int rows, int cols, double mu, double sigma) {
    default_random_engine generator(time(0));
    normal_distribution<double> distribution(mu, sigma);
    MatrixXd noise(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            noise(i, j) = distribution(generator);
            noise(i, j) = min(max(noise(i, j), -1.0), 1.0); // Limitar a [-1, 1]
        }
    }
    return noise;
}

// Filtro Kalman Extendido
void extendedKalmanFilter(MatrixXd& X, const VectorXd& Dk, const VectorXd& delta, double L, MatrixXd& Xk, MatrixXd& Pk) {
    int n = X.cols();
    MatrixXd Id = MatrixXd::Identity(3, 3);
    MatrixXd Hk = MatrixXd::Identity(3, 3);
    MatrixXd Qk, Rk;

    for (int i = 0; i < n; ++i) {
        if (X.col(i).isZero()) {
            Qk = MatrixXd::Identity(3, 3) * 1500;
            Rk = MatrixXd::Identity(3, 3) * 1e10;
        } else {
            Qk = MatrixXd::Identity(3, 3) * 15;
            Rk = MatrixXd::Identity(3, 3) * 0.1 * 0.1;
        }

        VectorXd Xk_i = Xk.col(i);
        Xk(0, i + 1) = Xk_i(0) + Dk(i) * cos(delta(i)) * cos(Xk_i(2));
        Xk(1, i + 1) = Xk_i(1) + Dk(i) * cos(delta(i)) * sin(Xk_i(2));
        Xk(2, i + 1) = Xk_i(2) + Dk(i) / L * sin(delta(i));

        MatrixXd Fk(3, 3);
        Fk << 1, 0, -Dk(i) * cos(delta(i)) * sin(Xk_i(2)),
              0, 1, Dk(i) * cos(delta(i)) * cos(Xk_i(2)),
              0, 0, 1;

        Pk = Fk * Pk * Fk.transpose() + Qk;

        // Actualización Kalman (opcional, si deseas medir contra Zk)
    }
}

int main() {
    // Datos de entrada
    MatrixXd data = readCSV("collectedData.csv", 1000, 5); // Ajusta el tamaño
    VectorXd ber_mea_x = data.col(0);
    VectorXd ber_mea_y = data.col(1);
    VectorXd ber_mea_theta = data.col(2);
    VectorXd ber_mea_throttle = data.col(3);
    VectorXd ber_mea_delta = data.col(4);

    // Filtrar NaN
    ber_mea_x = ber_mea_x.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_y = ber_mea_y.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_theta = ber_mea_theta.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_throttle = ber_mea_throttle.unaryExpr([](double v) { return isnan(v) ? 0 : v; });
    ber_mea_delta = ber_mea_delta.unaryExpr([](double v) { return isnan(v) ? 0 : v; });

    // Ruido Gaussiano
    int n = ber_mea_x.size();
    MatrixXd ruido_gaussiano = generateGaussianNoise(3, n, 0, 0.1);

    // Inicialización
    MatrixXd X = MatrixXd::Zero(3, n);
    X.row(0) = ber_mea_x.transpose();
    X.row(1) = ber_mea_y.transpose();
    X.row(2) = ber_mea_theta.transpose();
    X += ruido_gaussiano;

    MatrixXd Xk = MatrixXd::Zero(3, n + 1);
    Xk.col(0) << ber_mea_x(0), ber_mea_y(0), ber_mea_theta(0);
    MatrixXd Pk = MatrixXd::Identity(3, 3) * 10;

    double L = 0.155;
    VectorXd Dk = ber_mea_throttle;
    VectorXd delta = ber_mea_delta;

    // Filtro Kalman Extendido
    extendedKalmanFilter(X, Dk, delta, L, Xk, Pk);

    cout << "Trayectoria estimada Xk:\n" << Xk << endl;
    cout << "Covarianza final Pk:\n" << Pk << endl;

    return 0;
}
