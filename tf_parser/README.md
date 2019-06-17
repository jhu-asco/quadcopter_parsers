Run the install.sh script from this directory before compiling

To use this class in a program that also uses protobuf, you must use a specific tensorflow and protobuf version (https://github.com/tensorflow/tensorflow/issues/24976)

For tensorflow, use commit a6d8ffa with build options:
bazel build -c opt --copt=-mavx --define=grpc_no_ares=true //tensorflow/tools/lib_package:libtensorflow

Copy the built tf libraries to the clib folder of this package to overwrite the tf libraries from the install script.

For protobuf, use https://github.com/protocolbuffers/protobuf/releases/download/v3.6.0/protobuf-all-3.6.0.tar.gz
