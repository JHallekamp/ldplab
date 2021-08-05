#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "IPipeline.hpp"

void ldplab::rtscuda::IPipeline::stepSetup(
	const ldplab::SimulationState& sim_state)
{
}

void ldplab::rtscuda::IPipeline::finalizeOutput(RayTracingStepOutput& output)
{
}

ldplab::Mat3 ldplab::rtscuda::IPipeline::getRotationMatrix(
	double rx, 
	double ry, 
	double rz, 
	RotationOrder order)
{
	return ldplab::Mat3();
}

#endif

