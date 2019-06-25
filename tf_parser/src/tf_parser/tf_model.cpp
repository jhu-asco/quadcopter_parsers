#include "tf_parser/tf_model.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

TFModel::TFModel(std::string checkpoint_path) : 
    gaussian_(Eigen::Vector3f::Zero(), Eigen::Matrix3f::Zero(), false) {
  std::string graph_def_filename = checkpoint_path + "graph.pb";
  std::string checkpoint_filename = checkpoint_path + "checkpoint";

  std::string checkpoint_prefix = checkpointPrefix(checkpoint_filename);
  std::string checkpoint_prefix_path = checkpoint_path + checkpoint_prefix;

  if (!modelCreate(&model_, graph_def_filename.c_str())) 
    throw std::runtime_error("Failed to load graph definition");

  if (!modelCheckpoint(&model_, checkpoint_prefix_path.c_str(), RESTORE))
    throw std::runtime_error("Failed to load checkpoint variables");

}

TFModel::~TFModel() {
  modelDestroy(&model_);
}


void TFModel::predict(const State& state, Eigen::Vector3f& control, State& next_state, const double dt) {
  Vector5f state_eig;
  state_eig << float(state.rpy(0)), float(state.rpy(1)), state.a_b.cast<float>();
  
  Vector5f f;
  Vector15f g;
  modelPredict(&model_, state_eig.data(), control.data(), f.data(), g.data());

  Eigen::Matrix<float, 5, 5> cov = gToCovariance(g);

  // TODO: Just sample 3 independent gaussians and use lower triangular form
  gaussian_.setMean(f);
  gaussian_.setCovar(cov);
  Eigen::VectorXf f_rand = gaussian_.samples(1).col(0);

  next_state.rpy.head<2>() = state.rpy.head<2>() + dt * f_rand.head<2>().cast<double>();
  next_state.a_b = state.a_b + dt * f_rand.tail<3>().cast<double>();

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(state.rpy(2), Eigen::Vector3d::UnitZ());
  
  next_state.v = state.v + dt * R * next_state.a_b;
  next_state.p = state.p + dt * next_state.v;
}

int TFModel::modelCreate(model_t* model, const char* graph_def_filename) {
  model->status = TF_NewStatus();
  model->graph = TF_NewGraph();

  {
    // Create the session.
    TF_SessionOptions* opts = TF_NewSessionOptions();
    model->session = TF_NewSession(model->graph, opts, model->status);
    TF_DeleteSessionOptions(opts);
    if (!okay(model->status)) return 0;
  }

  TF_Graph* g = model->graph;

  {
    // Import the graph.
    TF_Buffer* graph_def = readFile(graph_def_filename);
    if (graph_def == NULL) return 0;
    printf("Read GraphDef of %zu bytes\n", graph_def->length);
    TF_ImportGraphDefOptions* opts = TF_NewImportGraphDefOptions();
    TF_GraphImportGraphDef(g, graph_def, opts, model->status);
    TF_DeleteImportGraphDefOptions(opts);
    TF_DeleteBuffer(graph_def);
    if (!okay(model->status)) return 0;
  }

  // Handles to the interesting operations in the graph.
  model->state_in.oper = TF_GraphOperationByName(g, "input/state_in");
  model->state_in.index = 0;
  model->control_in.oper = TF_GraphOperationByName(g, "input/controls_in");
  model->control_in.index = 0;
  model->f_output.oper = TF_GraphOperationByName(g, "f/hidden_to_output/f/hidden_to_output/dense_final/BiasAdd");
  model->f_output.index = 0;
  model->g_output.oper = TF_GraphOperationByName(g, "g/hidden_to_output/g/hidden_to_output/dense_final/BiasAdd");
  model->g_output.index = 0;
  model->batch_norm_phase.oper = TF_GraphOperationByName(g, "input/batch_normalization_phase");
  model->batch_norm_phase.index = 0;

  model->init_op = TF_GraphOperationByName(g, "init");
  model->train_op = TF_GraphOperationByName(g, "train");
  model->save_op = TF_GraphOperationByName(g, "save/control_dependency");
  model->restore_op = TF_GraphOperationByName(g, "save/restore_all");

  model->checkpoint_file.oper = TF_GraphOperationByName(g, "save/Const");
  model->checkpoint_file.index = 0;
  return 1;
}

Eigen::Matrix<float, 5, 5> TFModel::fillSpiral(const Vector15f& g) {
  Eigen::Matrix<float, 5, 5> cov_tri;
  cov_tri.setZero();
  std::vector<int> rows = {4, 0, 3, 1, 2};
  bool reverse = true;
  int i = 0;
  for(auto row : rows) {
    for(int col = 0; col <= row; col++) {
      if(reverse) {
        float val = (row == (row-col) ? exp(g(i++)) + 1e-4 : g(i++));
        cov_tri(row, row - col) = val;
      } else {
        float val = (row == col ? exp(g(i++)) + 1e-4 : g(i++));
        cov_tri(row, col) = val;
      }
    }
    reverse = !reverse;
  }
  return cov_tri;
}

Eigen::Matrix<float, 5, 5> TFModel::gToCovariance(const Vector15f& g) {
  // First convert g vec to lower triangular form
  /*
  Eigen::Matrix<float, 5, 5> cov_tri;
  int row = 0, col = 0;
  for(int i = 0; i < 15; i++) {
    if(i == (row + 1) * (row + 2) / 2) {
      row++;
      col = 0;
    }
    if(row == col) {
      cov_tri(row, col) = exp(g(i)) + 1e-4;
    } else {
      cov_tri(row, col) = g(i);
    }
    col++;
  }
  */
  auto cov_tri = fillSpiral(g);
  return cov_tri * cov_tri.transpose();
} 

int TFModel::modelPredict(model_t* model, float* state, float* control, float* f, float* g) {
  const int64_t state_dims[2] = {1, 5};
  const int64_t control_dims[2] = {1, 3};
  const size_t state_nbytes = 5 * sizeof(float);
  const size_t control_nbytes = 3 * sizeof(float);
  const size_t batch_norm_nbytes = 1 * sizeof(bool);
  const size_t f_nbytes = 5 * sizeof(float);
  const size_t g_nbytes = 15 * sizeof(float);
  const bool batch_norm[1] = {false};

  TF_Tensor* state_t = TF_AllocateTensor(TF_FLOAT, state_dims, 2, state_nbytes);
  TF_Tensor* control_t = TF_AllocateTensor(TF_FLOAT, control_dims, 2, control_nbytes);
  TF_Tensor* batch_norm_t = TF_AllocateTensor(TF_BOOL, NULL, 0, batch_norm_nbytes);
  memcpy(TF_TensorData(state_t), state, state_nbytes);
  memcpy(TF_TensorData(control_t), control, control_nbytes);
  memcpy(TF_TensorData(batch_norm_t), &batch_norm[0], batch_norm_nbytes);

  TF_Output inputs[3] = {model->state_in, model->control_in, model->batch_norm_phase};
  TF_Tensor* input_values[3] = {state_t, control_t, batch_norm_t};
  TF_Output outputs[4] = {model->f_output, model->g_output};
  TF_Tensor* output_values[4] = {NULL, NULL};

  TF_SessionRun(model->session, NULL, inputs, input_values, 3, outputs,
                output_values, 2,
                /* No target operations to run */
                NULL, 0, NULL, model->status);
  TF_DeleteTensor(state_t);
  TF_DeleteTensor(control_t);
  TF_DeleteTensor(batch_norm_t);
  if (!okay(model->status)) return 0;

  if (TF_TensorByteSize(output_values[0]) != f_nbytes) {
    fprintf(stderr,
            "ERROR: Expected predictions tensor to have %zu bytes, has %zu\n",
            f_nbytes, TF_TensorByteSize(output_values[0]));
    TF_DeleteTensor(output_values[0]);
    return 0;
  }
  if (TF_TensorByteSize(output_values[1]) != g_nbytes) {
    fprintf(stderr,
            "ERROR: Expected predictions tensor to have %zu bytes, has %zu\n",
            g_nbytes, TF_TensorByteSize(output_values[1]));
    TF_DeleteTensor(output_values[1]);
    return 0;
  }

  memcpy(f, TF_TensorData(output_values[0]), f_nbytes);
  TF_DeleteTensor(output_values[0]);

  memcpy(g, TF_TensorData(output_values[1]), g_nbytes);
  TF_DeleteTensor(output_values[1]);

  return 1;
}

int TFModel::modelCheckpoint(model_t* model, const char* checkpoint_prefix, int type) {
  TF_Tensor* t = scalarStringTensor(checkpoint_prefix, model->status);
  if (!okay(model->status)) {
    TF_DeleteTensor(t);
    return 0;
  }
  TF_Output inputs[1] = {model->checkpoint_file};
  TF_Tensor* input_values[1] = {t};
  const TF_Operation* op[1] = {type == SAVE ? model->save_op
                                            : model->restore_op};
  TF_SessionRun(model->session, NULL, inputs, input_values, 1,
                /* No outputs */
                NULL, NULL, 0,
                /* The operation */
                op, 1, NULL, model->status);
  TF_DeleteTensor(t);

  return okay(model->status);
}

std::string TFModel::checkpointPrefix(std::string checkpoint_fn) {
  int checkpoint_str_sz = strlen("model_checkpoint_path");
  int fd = open(checkpoint_fn.c_str(), 0);
  if (fd < 0) {
    throw std::runtime_error("Failed to read file");
  }
  struct stat stat;
  if (fstat(fd, &stat) != 0) {
    throw std::runtime_error("Failed to read file");
  }
  char* data = (char*)malloc(stat.st_size);
  ssize_t nread = read(fd, data, stat.st_size);
  if (nread < 0) {
    free(data);
    throw std::runtime_error("Failed to read file");
  }
  if (nread != stat.st_size) {
    fprintf(stderr, "read %zd bytes, expected to read %zd\n", nread,
            stat.st_size);
    free(data);
    throw std::runtime_error("Failed to read file");
  }
  if (nread < checkpoint_str_sz) {
    fprintf(stderr, "Checkpoint file smaller than expected\n");
    free(data);
    throw std::runtime_error("Failed to read file");
  }

  /**
    The checkpoint protobuf file should have the first line as:
      model_checkpoint_path: "variable_prefix"
    The below checks for the variable name first then extracts the corresponding value
  **/
  char* checkpoint_str = (char*)malloc(checkpoint_str_sz + 1);
  memcpy(checkpoint_str, data, checkpoint_str_sz);
  checkpoint_str[checkpoint_str_sz] = '\0';
  if (strcmp("model_checkpoint_path", checkpoint_str) != 0) {
    printf("Did not find checkpoint variable!\n");
    printf("Found %s\n", checkpoint_str);

    free(checkpoint_str);
    free(data);
    throw std::runtime_error("Failed to read file");
  }

  free(checkpoint_str);
  printf("Found checkpoint variable!\n");

  int str_idx_start = checkpoint_str_sz;
  while(data[str_idx_start] != '\"') {
    str_idx_start++;
    if(str_idx_start == nread) {
      free(data);
      throw std::runtime_error("File format incorrect");
    }
  } 

  int str_idx_end = str_idx_start + 1;
  while(data[str_idx_end] != '\"') {
    str_idx_end++;
    if(str_idx_end == nread) {
      free(data);
      throw std::runtime_error("File format incorrect");
    }
  } 

  int checkpoint_prefix_sz = str_idx_end - (str_idx_start + 1);
  char* checkpoint_prefix = (char*)malloc(checkpoint_prefix_sz + 1);
  memcpy(checkpoint_prefix, data + str_idx_start + 1, checkpoint_prefix_sz);
  checkpoint_prefix[checkpoint_prefix_sz] = '\0';

  std::string checkpoint_prefix_out(checkpoint_prefix);

  free(checkpoint_prefix);
  free(data);
  return checkpoint_prefix_out;
}

void TFModel::modelDestroy(model_t* model) {
  if(model->session)
    TF_DeleteSession(model->session, model->status);
  okay(model->status);
  if(model->graph)
    TF_DeleteGraph(model->graph);
  if(model->status)
    TF_DeleteStatus(model->status);
}

TF_Buffer* TFModel::readFile(const char* filename) {
  int fd = open(filename, 0);
  if (fd < 0) {
    perror("failed to open file: ");
    return NULL;
  }
  struct stat stat;
  if (fstat(fd, &stat) != 0) {
    perror("failed to read file: ");
    return NULL;
  }
  char* data = (char*)malloc(stat.st_size);
  ssize_t nread = read(fd, data, stat.st_size);
  if (nread < 0) {
    perror("failed to read file: ");
    free(data);
    return NULL;
  }
  if (nread != stat.st_size) {
    fprintf(stderr, "read %zd bytes, expected to read %zd\n", nread,
            stat.st_size);
    free(data);
    return NULL;
  }
  TF_Buffer* ret = TF_NewBufferFromString(data, stat.st_size);
  free(data);
  return ret;
}

TF_Tensor* TFModel::scalarStringTensor(const char* str, TF_Status* status) {
  size_t nbytes = 8 + TF_StringEncodedSize(strlen(str));
  TF_Tensor* t = TF_AllocateTensor(TF_STRING, NULL, 0, nbytes);
  void* data = TF_TensorData(t);
  memset(data, 0, 8);  // 8-byte offset of first string.
  TF_StringEncode(str, strlen(str), (char*)data + 8, nbytes - 8, status);
  return t;
}

int TFModel::okay(TF_Status* status) {
  if (TF_GetCode(status) != TF_OK) {
    fprintf(stderr, "ERROR: %s\n", TF_Message(status));
    return 0;
  }
  return 1;
}
