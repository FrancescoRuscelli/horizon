#include <gtest/gtest.h>
#include "../src/wrapped_function.h"

namespace{

class testCasadiUtils: public ::testing::Test
{
protected:
    testCasadiUtils(){

    }

    virtual ~testCasadiUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testCasadiUtils, testToCasadiMatrix)
{
    Eigen::MatrixXd E;
    E.setRandom(5,6);

    casadi::DM C;
    casadi_utils::toCasadiMatrix(E, C);

    EXPECT_EQ(E.rows(), C.rows());
    EXPECT_EQ(E.cols(), C.columns());

    for(unsigned int i = 0; i < E.rows(); ++i)
    {
        for(unsigned int j = 0; j < E.cols(); ++j)
            EXPECT_DOUBLE_EQ(E(i,j), double(C(i,j)));
    }

    std::cout<<"E: \n"<<E<<std::endl;
    std::cout<<"C: "<<C<<std::endl;

    Eigen::VectorXd e;
    e.setRandom(7);

    casadi::DM c;
    casadi_utils::toCasadiMatrix(e, c);

    EXPECT_EQ(e.rows(), c.rows());
    EXPECT_EQ(e.cols(), c.columns());

    for(unsigned int i = 0; i < e.size(); ++i)
        EXPECT_DOUBLE_EQ(e[i], double(c(i)));

    std::cout<<"e: "<<e.transpose()<<std::endl;
    std::cout<<"c: "<<c<<std::endl;

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
