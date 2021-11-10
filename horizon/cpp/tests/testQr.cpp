#include <Eigen/Dense>
#include <iostream>
#include <gtest/gtest.h>


TEST(testQr, rank)
{
    Eigen::MatrixXd A, R;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr;

    for(int i = 0; i < 100000; i++)
    {
        A = Eigen::MatrixXd::Random(10, 3)*Eigen::MatrixXd::Random(3, 5);
        qr.compute(A);
        R = qr.matrixQR().triangularView<Eigen::Upper>();

        EXPECT_LT(std::fabs(R(3, 3)), 1e-9);
        EXPECT_LT(std::fabs(R(4, 4)), 1e-9);

        A = Eigen::MatrixXd::Random(5, 3)*Eigen::MatrixXd::Random(3, 10);
        qr.compute(A);
        R = qr.matrixQR().triangularView<Eigen::Upper>();

        EXPECT_LT(std::fabs(R(3, 3)), 1e-9);
        EXPECT_LT(std::fabs(R(4, 4)), 1e-9);
    }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
