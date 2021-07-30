#include <casadi/casadi.hpp>
#include <Eigen/Dense>

namespace horizon
{

class IterativeLQR
{

public:

    IterativeLQR(casadi::Function fdyn,
                 int N);

    void setIntermediateCost(const std::vector<casadi::Function>& inter_cost);

    void setFinalCost(const casadi::Function& final_cost);

protected:

private:

    void linearize_quadratize();
    void backward_pass();
    void backward_pass_iter(int i);
    void compute_defect(int i, Eigen::VectorXd& d);
    Eigen::Ref<Eigen::VectorXd> state(int i);
    Eigen::Ref<Eigen::VectorXd> input(int i);

    struct Dynamics
    {

    public:

        // dynamics function
        casadi::Function f;
        casadi::Function df;

        // df/dx
        Eigen::MatrixXd A;

        // df/du
        Eigen::MatrixXd B;

        // defect (or gap)
        Eigen::VectorXd d;

        Dynamics(int nx, int nu);

        void linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);

    };

    struct Cost
    {
        // original cost
        casadi::Function l;
        casadi::Function dl;
        casadi::Function ddl;

        /* Quadratized cost */
        Eigen::MatrixXd Q;
        Eigen::VectorXd q;
        Eigen::MatrixXd R;
        Eigen::VectorXd r;
        Eigen::MatrixXd P;

        Cost(int nx, int nu);

        void setCost(const casadi::Function& cost);

        void quadratize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
    };

    struct BackwardPassResult
    {
        Eigen::MatrixXd Lfb;
        Eigen::VectorXd du_ff;

        BackwardPassResult(int nx, int nu);
    };

    struct Temporaries
    {
        Eigen::MatrixXd s_plus_S_d;
        Eigen::MatrixXd S_A;

        Eigen::MatrixXd Huu;
        Eigen::MatrixXd Hux;
        Eigen::MatrixXd Hxx;

        Eigen::VectorXd hx;
        Eigen::VectorXd hu;

        Eigen::MatrixXd huHux;

        Eigen::LLT<Eigen::MatrixXd> llt;
    };

    int _nx;
    int _nu;
    int _N;

    casadi::Function _f;
    casadi::Function _df;

    std::vector<Cost> _cost;
    std::vector<Cost> _value;
    std::vector<Dynamics> _dyn;
    std::vector<BackwardPassResult> _res;
    Eigen::MatrixXd _xtrj;
    Eigen::MatrixXd _utrj;

    Temporaries tmp;
};

}
