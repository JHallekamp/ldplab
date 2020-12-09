#include "InitialStage.hpp"

ldplab::rtscpu::InitialStage::InitialStage(
    std::shared_ptr<Context> context)
    :
    m_context{ context }
{ }

void ldplab::rtscpu::InitialStage::executeSetup()
{
}

void ldplab::rtscpu::InitialStage::execute(size_t particle)
{
}