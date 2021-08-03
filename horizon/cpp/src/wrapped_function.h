#ifndef WRAPPED_FUNCTION_H
#define WRAPPED_FUNCTION_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>

namespace casadi_utils
{

class WrappedFunction
{

public:

    WrappedFunction() = default;
    WrappedFunction(casadi::Function f);

    void setInput(int i, Eigen::Ref<const Eigen::VectorXd> xi);
    void call();
    const Eigen::MatrixXd& getOutput(int i) const;
    casadi::Function& function();

    bool is_valid() const;

private:

    void csc_to_matrix(const casadi::Sparsity& sp,
                       const std::vector<double>& data,
                       Eigen::MatrixXd& matrix);

    std::vector<const double *> _in_buf;
    std::vector<std::vector<double>> _out_data;
    std::vector<Eigen::MatrixXd> _out_matrix;
    std::vector<double *> _out_buf;
    std::vector<casadi_int> _iw;
    std::vector<double> _dw;

    casadi::Function _f;
};

}

#endif // WRAPPED_FUNCTION_H
