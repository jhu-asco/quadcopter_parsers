#!/usr/bin/bash
mkdir clib
curl -L "https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-linux-x86_64-1.4.0.tar.gz" | tar -C clib -xz
