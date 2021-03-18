#include "ComputeHelper.hpp"

size_t ldplab::utils::ComputeHelper::getNumWorkGroups(
    size_t num_threads, size_t local_size)
{
    return (num_threads / local_size) + (num_threads % local_size ? 1 : 0);
}
