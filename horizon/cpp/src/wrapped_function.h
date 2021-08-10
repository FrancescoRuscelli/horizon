#ifndef WRAPPED_FUNCTION_H
#define WRAPPED_FUNCTION_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace casadi_utils
{

template <typename T, int r_size, int c_size>
void toCasadiMatrix(const Eigen::Matrix<T, r_size, c_size>& E, casadi::Matrix<T>& C)
{
    C.resize(E.rows(), E.cols());
    C = casadi::Matrix<T>::zeros(E.rows(), E.cols());
    std::memcpy(C.ptr(), E.data(), sizeof(T)*E.rows()*E.cols());
}

template <typename T, int r_size, int c_size>
void toEigen(const casadi::Matrix<T>& C, Eigen::Matrix<T, r_size, c_size>& E)
{
    E.setZero(C.rows(), C.columns());
    std::memcpy(E.data(), C.ptr(), sizeof(T)*C.rows()*C.columns());
}


class WrappedFunction
{

public:

    WrappedFunction() = default;
    WrappedFunction(casadi::Function f);

    void setInput(int i, Eigen::Ref<const Eigen::VectorXd> xi);
    void call(bool sparse = false);
    const Eigen::MatrixXd& getOutput(int i) const;
    const Eigen::SparseMatrix<double>& getSparseOutput(int i) const;
    casadi::Function& function();

    bool is_valid() const;

private:

    void csc_to_matrix(const casadi::Sparsity& sp,
                       const std::vector<double>& data,
                       Eigen::MatrixXd& matrix);
    void csc_to_sparse_matrix(const casadi::Sparsity& sp,
                              const std::vector<double>& data,
                              Eigen::SparseMatrix<double>& matrix);

    std::vector<const double *> _in_buf;
    std::vector<std::vector<double>> _out_data;
    std::vector<Eigen::MatrixXd> _out_matrix;
    std::vector<Eigen::SparseMatrix<double> > _out_matrix_sparse;
    std::vector<double *> _out_buf;
    std::vector<casadi_int> _iw;
    std::vector<double> _dw;

    casadi::Function _f;
};

}

#endif // WRAPPED_FUNCTION_H
