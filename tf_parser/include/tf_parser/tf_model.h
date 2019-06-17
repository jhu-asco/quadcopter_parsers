#pragma once

#include <string>
#include <Eigen/Dense>
#include "tf_parser/state.h"
#include <tensorflow/c/c_api.h>
#include <gtest/gtest.h>

class TFModel {
public:
  TFModel(std::string checkpoint_path);
  ~TFModel();
  void predict(const State& state, Eigen::Vector3f& control, State& next_state, const double dt);

private:
  FRIEND_TEST(TFModelTests, Dynamics);

  typedef Eigen::Matrix<float, 5, 1> Vector5f;
  typedef Eigen::Matrix<float, 15, 1> Vector15f;

  typedef struct model_t {
    TF_Graph* graph;
    TF_Session* session;
    TF_Status* status;
  
    TF_Output batch_norm_phase, state_in, control_in, f_output, g_output;
  
    TF_Operation *init_op, *train_op, *save_op, *restore_op;
    TF_Output checkpoint_file;

    model_t(): graph(NULL), session(NULL), status(NULL),
      init_op(NULL), train_op(NULL), save_op(NULL), restore_op(NULL) {}
  } model_t;

  int modelCreate(model_t* model, const char* graph_def_filename);
  void modelDestroy(model_t* model);
  int modelPredict(model_t* model, float*, float*, float*, float*);
  int modelCheckpoint(model_t* model, const char* checkpoint_prefix, int type);
  enum SaveOrRestore { SAVE, RESTORE };

  int okay(TF_Status* status);
  TF_Buffer* readFile(const char* filename);
  std::string checkpointPrefix(std::string checkpoint_fn);
  TF_Tensor* scalarStringTensor(const char* str, TF_Status* status);
  Eigen::Matrix<float, 5, 5> gToCovariance(const Vector15f& g);

  model_t model_;
};
