#!/bin/sh

cmake -DLDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING=ON -DLDPLAB_BUILD_OPTION_ENABLE_PROFILING=ON -DLDPLAB_BUILD_OPTION_ENABLE_RTSCUDA=ON -DBUILD_LDPLAB_TEST_PROJECTS=ON -S . -B out/build
