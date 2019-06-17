#include <gtest/gtest.h>
#include <tf_parser/tf_model.h>

typedef Eigen::Matrix<float, 5, 1> Vector5f;
typedef Eigen::Matrix<float, 15, 1> Vector15f;

TEST(TFModelTests, BaseCtor) {
  ASSERT_NO_THROW(TFModel("/home/matt/spo/python/deep/model/logs/variable_log/2019-06-05_11-12-41/"));
}

TEST(TFModelTests, Dynamics) {
  TFModel model("/home/matt/spo/python/deep/model/logs/variable_log/2019-06-05_11-12-41/");
  Vector5f f;
  Vector15f g;
  Vector5f state;
  Eigen::Vector3f control;
  state << 0, 0, 0, 0, 0;
  control << 0, 0, 0;
  model.modelPredict(&model.model_, state.data(), control.data(), f.data(), g.data());

  Vector5f expected_f;
  expected_f << -0.4805804, -0.63574195, -4.304112, 6.010012, -36.603912;
  Vector15f expected_g;
  expected_g << 15.404918, -3.6593773, 4.5528617, 1.7883033,
          2.8935068, -9.640302, -0.40782666, 0.3061283,
         -1.5039382, 1.7642134, 0.20243622, -10.624643,
         -2.102911, -4.050644, 2.1983354;

  for(int i = 0; i < f.size(); i++) {
    ASSERT_NEAR(f(i), expected_f(i), 1e-6);
  }
  for(int i = 0; i < g.size(); i++) {
    ASSERT_NEAR(g(i), expected_g(i), 1e-6);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
