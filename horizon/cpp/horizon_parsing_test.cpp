#include <yaml-cpp/yaml.h>
#include <vector>
#include <boost/lexical_cast.hpp>
#include "src/ilqr.h"

#include "src/wrapped_function.h"

namespace horizon
{

class Problem
{

public:

    void from_yaml(YAML::Node problem_yaml);

    void print();

    void update_bounds();

    struct Variable
    {
        typedef std::shared_ptr<Variable> Ptr;

        std::string name;
        casadi::SX sym;
        Eigen::MatrixXd lb, ub, value, initial_guess;

        int size();
    };

    struct Function
    {
        typedef std::shared_ptr<Function> Ptr;

        std::string name;
        casadi::Function fun;
        Eigen::MatrixXd lb, ub;
        std::vector<int> nodes;
    };

    std::map<std::string, Variable::Ptr> var_map, param_map;

    std::map<std::string, Function::Ptr> fun_map, cost_map, constr_map;

    std::vector<Variable::Ptr> state_vec, input_vec;

    casadi::SX x, u;
    Eigen::MatrixXd xlb, xub, ulb, uub, x_ini, u_ini;

    casadi::Function dynamics;

    int N;

private:

    Variable::Ptr yaml_to_variable(YAML::Node item);
    Function::Ptr yaml_to_function(std::pair<YAML::Node, YAML::Node> item,
                                   std::string outname);
};

Problem::Variable::Ptr Problem::yaml_to_variable(YAML::Node item)
{
    auto v = std::make_shared<Variable>();

    auto var_data = item;
    auto name = var_data["name"].as<std::string>();
    v->name = name;

    int size = var_data["size"].as<int>();
    v->sym = casadi::SX::sym(name, size);

    try
    {
        auto lb = var_data["lb"].as<std::vector<double>>();
        auto ub = var_data["ub"].as<std::vector<double>>();
        auto ini = var_data["initial_guess"].as<std::vector<double>>();

        v->lb = Eigen::MatrixXd::Map(lb.data(), size, lb.size()/size);
        v->ub = Eigen::MatrixXd::Map(ub.data(), size, ub.size()/size);
        v->initial_guess = Eigen::MatrixXd::Map(ini.data(), size, ini.size()/size);
    }
    catch(YAML::Exception&)
    {

    }

    try
    {
        auto values = var_data["values"].as<std::vector<double>>();
        v->value = Eigen::MatrixXd::Map(values.data(), size, values.size()/size);
    }
    catch(YAML::Exception&)
    {

    }

    return v;
}

Problem::Function::Ptr Problem::yaml_to_function(std::pair<YAML::Node, YAML::Node> item,
                                                 std::string outname)
{
    auto fun = std::make_shared<Function>();
    auto name = item.first.as<std::string>();
    auto var_data = item.second;

    fun->name = name;
    fun->nodes = var_data["nodes"].as<std::vector<int>>();

    // wrap f as f(x, y, p...)
    auto f = casadi::Function::deserialize(var_data["function"].as<std::string>());

    // compute function value as SX
    std::vector<casadi::SX> fun_input;
    std::vector<casadi::SX> fun_input_param;
    std::vector<std::string> param_names;
    for(int i = 0; i < f.n_in(); i++)
    {
        fun_input.push_back(var_map.at(f.name_in(i))->sym);

        if(param_map.count(f.name_in(i)))
        {
            param_names.push_back(f.name_in(i));
            fun_input_param.push_back(fun_input.back());
        }
    }

    // compute symbolic value
    casadi::SX fn_value = f(fun_input)[0];

    // wrap function with required signature
    std::vector<casadi::SX> wrapped_inputs = {x, u};
    wrapped_inputs.insert(wrapped_inputs.end(),
                          fun_input_param.begin(),
                          fun_input_param.end());

    std::vector<std::string> wrapped_names = {"x", "u"};
    wrapped_names.insert(wrapped_names.end(),
                         param_names.begin(),
                         param_names.end());

    fun->fun = casadi::Function(f.name(),
                                wrapped_inputs, {fn_value},
                                wrapped_names, {outname});

    // bounds
    try
    {
        auto lb = var_data["lb"].as<std::vector<double>>();
        auto ub = var_data["ub"].as<std::vector<double>>();

        fun->lb = Eigen::MatrixXd::Map(lb.data(), f.size1_out(0), lb.size()/f.size1_out(0));
        fun->ub = Eigen::MatrixXd::Map(ub.data(), f.size1_out(0), ub.size()/f.size1_out(0));
    }
    catch(YAML::Exception& e)
    {

    }

    return fun;

}

void Problem::from_yaml(YAML::Node problem_yaml)
{
    // nodes
    N = problem_yaml["n_nodes"].as<int>();

    // retrieve state
    std::vector<casadi::SX> x_sx_vec;
    for(auto item : problem_yaml["state"])
    {
        auto v = yaml_to_variable(item);

        std::cout << "state " << v->name << "[" << v->sym.size1() << "]\n" <<
                     "=======\n";

        state_vec.push_back(v);
        var_map[v->name] = v;
        x_sx_vec.push_back(v->sym);

    }

    // retrieve input
    std::vector<casadi::SX> u_sx_vec;
    for(auto item : problem_yaml["input"])
    {
        auto v = yaml_to_variable(item);

        std::cout << "input " << v->name << "[" << v->sym.size1() << "]\n" <<
                     "=======\n";

        input_vec.push_back(v);
        var_map[v->name] = v;
        u_sx_vec.push_back(v->sym);

    }

    // retrieve param
    for(auto item : problem_yaml["param"])
    {
        auto v = yaml_to_variable(item);

        std::cout << "param " << v->name << "[" << v->sym.size1() << "]\n" <<
                     "=======\n";

        var_map[v->name] = param_map[v->name] = v;
    }

    // dynamics
    dynamics = casadi::Function::deserialize(problem_yaml["dynamics"].as<std::string>());
    x = casadi::SX::vertcat(x_sx_vec);
    u = casadi::SX::vertcat(u_sx_vec);

    // retrive costs
    for(auto item : problem_yaml["cost"])
    {
        auto f = yaml_to_function(item, "l");

        cost_map[f->name] = fun_map[f->name] = f;
    }

    // retrive constraints
    for(auto item : problem_yaml["constraint"])
    {
        auto f = yaml_to_function(item, "h");

        constr_map[f->name] = fun_map[f->name] = f;
    }

    // compute state and input bounds
    update_bounds();
}

void Problem::print()
{
    std::cout << "n_nodes = " << N << "\n\n";
    std::cout << "dynamics: " << dynamics << "\n\n";
    std::cout << "state = " << x << "\n\n";
    std::cout << "input = " << u << "\n\n";
}

void Problem::update_bounds()
{
    xlb.resize(x.size1(), N+1);
    xub.resize(x.size1(), N+1);
    ulb.resize(u.size1(), N);
    uub.resize(u.size1(), N);
    x_ini.resize(x.size1(), N+1);
    u_ini.resize(u.size1(), N);

    int row = 0;
    for(auto xi : state_vec)
    {
        xlb.middleRows(row, xi->size()) = xi->lb;
        xub.middleRows(row, xi->size()) = xi->ub;
        x_ini.middleRows(row, xi->size()) = xi->initial_guess;
        row += xi->size();
    }

    row = 0;
    for(auto ui : input_vec)
    {
        ulb.middleRows(row, ui->size()) = ui->lb;
        uub.middleRows(row, ui->size()) = ui->ub;
        u_ini.middleRows(row, ui->size()) = ui->initial_guess;
        row += ui->size();
    }
}

int Problem::Variable::size()
{
    return sym.size1();
}

}























int main(int argc, char **argv)
{
    std::string path_to_yaml = argv[1];

    auto problem_yaml = YAML::LoadFile(path_to_yaml);

    horizon::Problem p;
    p.from_yaml(problem_yaml);
    p.print();

    horizon::IterativeLQR::OptionDict opt;
    for(auto item : problem_yaml["solver"]["opt"])
    {
        auto optname = item.first.as<std::string>();
        auto optval= item.second.as<std::string>();

        try
        {
            opt[optname] = boost::lexical_cast<int>(optval);
            std::cout << optval << " is int \n";
            continue;
        }
        catch(boost::bad_lexical_cast& e){}

        try
        {
            opt[optname] = boost::lexical_cast<double>(optval);
            std::cout << optval << " is double \n";
            continue;
        }
        catch(boost::bad_lexical_cast& e){}

        if(optval == "true") opt[optname] = 1;
        else if(optval == "false") opt[optname] = 0;
        else opt[optname] = optval;
    }

    horizon::IterativeLQR ilqr(p.dynamics, p.N, opt);

    for(auto item : p.cost_map)
    {
        ilqr.setCost(item.second->nodes,
                     item.second->fun);
    }

    for(auto item : p.constr_map)
    {
        ilqr.setConstraint(item.second->nodes,
                           item.second->fun);
    }

    ilqr.setStateBounds(p.xlb, p.xub);
    ilqr.setInputBounds(p.ulb, p.uub);
    ilqr.setStateInitialGuess(p.x_ini);
    ilqr.setInputInitialGuess(p.u_ini);
    ilqr.setInitialState(p.xlb.col(0));

    ilqr.setIterationCallback(
                [](const horizon::IterativeLQR::ForwardPassResult& res)
    {
        res.print();
        return true;
    });

    ilqr.solve(1000);

}
