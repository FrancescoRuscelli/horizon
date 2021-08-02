#include "wrapped_function.h"

using namespace casadi_utils;


WrappedFunction::WrappedFunction(casadi::Function f):
    _f(f)
{
    // resize work vectors
    _iw.assign(_f.sz_iw(), 0);
    _dw.assign(_f.sz_w(), 0.);

    // resize input buffers
    _in_buf.assign(_f.n_in(), nullptr);

    // create memory for output data
    for(int i = 0; i < _f.n_out(); i++)
    {
        const auto& sp = _f.sparsity_out(i);

        // allocate memory for all nonzero elements of this output
        _out_data.emplace_back(sp.nnz(), 0.);

        // push the allocated buffer address to a vector
        _out_buf.push_back(_out_data.back().data());

        // allocate a zero dense matrix to store the output
        _out_matrix.emplace_back(Eigen::MatrixXd::Zero(sp.size1(), sp.size2()));
    }
}

void WrappedFunction::setInput(int i, Eigen::Ref<const Eigen::VectorXd> xi)
{
    if(xi.size() != _f.size1_in(i))
    {
        throw std::invalid_argument(_f.name() + ": input size mismatch");
    }

    _in_buf[i] = xi.data();
}

void WrappedFunction::call()
{
    // call function (allocation-free)
    _f(_in_buf.data(), _out_buf.data(), _iw.data(), _dw.data(), 0);

    // copy all outputs to dense matrices
    for(int i = 0; i < _f.n_out(); i++)
    {
        csc_to_matrix(_f.sparsity_out(i),
                      _out_data[i],
                      _out_matrix[i]);
    }
}

const Eigen::MatrixXd& WrappedFunction::getOutput(int i) const
{
    return _out_matrix[i];
}

casadi::Function& WrappedFunction::function()
{
    return _f;
}

void WrappedFunction::csc_to_matrix(const casadi::Sparsity& sp,
                                    const std::vector<double>& data,
                                    Eigen::MatrixXd& matrix)
{
    // if dense output, do copy assignment which should be
    // faster
    if(sp.is_dense())
    {
        matrix = Eigen::MatrixXd::Map(data.data(),
                                      matrix.rows(),
                                      matrix.cols());
        return;
    }

    int col_j = 0;
    for(int k = 0; k < sp.nnz(); k++)
    {
        // current elem row index
        int row_i = sp.row(k);

        // update current elem col index
        if(k == sp.colind(col_j + 1))
        {
            col_j++;
        }

        // copy data
        matrix(row_i, col_j) =  data[k];
    }
}
