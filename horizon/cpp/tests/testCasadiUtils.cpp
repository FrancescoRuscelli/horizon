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

    Eigen::MatrixXd EE;
    casadi_utils::toEigen(C, EE);

    EXPECT_EQ(EE.rows(), C.rows());
    EXPECT_EQ(EE.cols(), C.columns());

    for(unsigned int i = 0; i < EE.rows(); ++i)
    {
        for(unsigned int j = 0; j < EE.cols(); ++j)
            EXPECT_DOUBLE_EQ(EE(i,j), double(C(i,j)));
    }

    std::cout<<"E: \n"<<E<<std::endl;
    std::cout<<"EE: \n"<<EE<<std::endl;
    std::cout<<"C: "<<C<<std::endl;

    std::cout<<"C: "<<C<<std::endl;

    Eigen::VectorXd e;
    e.setRandom(7);

    casadi::DM c;
    casadi_utils::toCasadiMatrix(e, c);

    EXPECT_EQ(e.rows(), c.rows());
    EXPECT_EQ(e.cols(), c.columns());

    for(unsigned int i = 0; i < e.size(); ++i)
        EXPECT_DOUBLE_EQ(e[i], double(c(i)));

    Eigen::VectorXd ee;
    casadi_utils::toEigen(c, ee);

    EXPECT_EQ(ee.rows(), c.rows());
    EXPECT_EQ(ee.cols(), c.columns());

    for(unsigned int i = 0; i < ee.size(); ++i)
        EXPECT_DOUBLE_EQ(ee[i], double(c(i)));


    std::cout<<"e: "<<e.transpose()<<std::endl;
    std::cout<<"ee: "<<ee.transpose()<<std::endl;
    std::cout<<"c: "<<c<<std::endl;

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
