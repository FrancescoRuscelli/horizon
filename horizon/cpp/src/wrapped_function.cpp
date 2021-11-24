#include "wrapped_function.h"

using namespace casadi_utils;


WrappedFunction::WrappedFunction(casadi::Function f)
{
    *this = f;
}

WrappedFunction &WrappedFunction::operator=(casadi::Function f)
{
    if(f.is_null())
    {
        return *this;
    }

    _f = f;

//    if(f.sz_arg() != f.n_in() ||
//            f.sz_res() != f.n_out())
//    {
//        throw std::runtime_error("f.sz_arg() != f.n_in() || f.sz_res() != f.n_out() => contact the developers!!!");
//    }

    // resize work vectors
    _iw.assign(_f.sz_iw(), 0);
    _dw.assign(_f.sz_w(), 0.);

    // resize input buffers (note: sz_arg might be > n_in!!)
    _in_buf.assign(_f.sz_arg(), nullptr);

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

        //allocate a zero sparse matrix to store the output
        _out_matrix_sparse.emplace_back(Eigen::SparseMatrix<double>(sp.size1(), sp.size2()));

        // save sparsity pattern for i-th output
        std::vector<casadi_int> rows, cols;
        sp.get_triplet(rows, cols);
        _rows.push_back(rows);
        _cols.push_back(cols);
    }

    for(int i = _f.n_out(); i < _f.sz_res(); i++)
    {
        _out_buf.push_back(nullptr);
    }

    return *this;
}

WrappedFunction::WrappedFunction(const WrappedFunction & other)
{
    *this = other._f;
}

void WrappedFunction::setInput(int i, Eigen::Ref<const Eigen::VectorXd> xi)
{
    if(xi.size() != _f.size1_in(i))
    {
        throw std::invalid_argument(_f.name() + ": input size mismatch");
    }

    _in_buf[i] = xi.data();
}

void WrappedFunction::call(bool sparse)
{
    // call function (allocation-free)
    casadi_int mem = _f.checkout();
    _f(_in_buf.data(), _out_buf.data(), _iw.data(), _dw.data(), mem);

    // copy all outputs to dense matrices
    for(int i = 0; i < _f.n_out(); ++i)
    {

        if(sparse)
        {
            csc_to_sparse_matrix(_f.sparsity_out(i),
                                 _rows[i], _cols[i],
                                         _out_data[i],
                                         _out_matrix_sparse[i]);
        }
        else
        {
            csc_to_matrix(_f.sparsity_out(i),
                          _rows[i], _cols[i],
                          _out_data[i],
                          _out_matrix[i]);
        }

    }

    // release mem (?)
    _f.release(mem);
}

const Eigen::MatrixXd& WrappedFunction::getOutput(int i) const
{
    return _out_matrix[i];
}

const Eigen::SparseMatrix<double>& WrappedFunction::getSparseOutput(int i) const
{
    return _out_matrix_sparse[i];
}

Eigen::MatrixXd& WrappedFunction::out(int i)
{
    return _out_matrix[i];
}

casadi::Function& WrappedFunction::function()
{
    return _f;
}

const casadi::Function &WrappedFunction::function() const
{
    return _f;
}

bool WrappedFunction::is_valid() const
{
    return !_f.is_null();
}

void WrappedFunction::csc_to_sparse_matrix(const casadi::Sparsity& sp,
                                           const std::vector<casadi_int>&  sp_rows,
                                           const std::vector<casadi_int>&  sp_cols,
                                           const std::vector<double>& data,
                                           Eigen::SparseMatrix<double>& matrix)
{
    std::vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(data.size());
    for(unsigned int i = 0; i < data.size(); ++i)
        triplet_list.push_back(Eigen::Triplet<double>(sp_rows[i], sp_cols[i], data[i]));

    matrix.setFromTriplets(triplet_list.begin(), triplet_list.end());
}

void WrappedFunction::csc_to_matrix(const casadi::Sparsity& sp,
                                    const std::vector<casadi_int>&  sp_rows,
                                    const std::vector<casadi_int>&  sp_cols,
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


    for(int k = 0; k < sp.nnz(); k++)
    {
        // current elem row index
        int row_i = sp_rows[k];
        int col_j = sp_cols[k];

        // copy data
        matrix(row_i, col_j) =  data[k];
    }
}

