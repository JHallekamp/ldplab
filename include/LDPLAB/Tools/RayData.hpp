#ifndef WWU_LDPLAB_TOOLS_RAY_DATA_HPP
#define WWU_LDPLAB_TOOLS_RAY_DATA_HPP

#include <LDPLAB/Geometry.hpp>
#include <vector>
#include <fstream>

namespace ldplab
{
	namespace tools
	{
		struct Segment
		{
			Vec3 origin;
			Vec3 end;
		};

		struct IRayBlock
		{
			enum class Type { inner_particle, outer_particle };
			virtual ~IRayBlock() { }
			virtual Type type() const = 0;
			virtual void write(std::ofstream stream) = 0;
			virtual size_t byteSize() = 0; 
		};

		struct InnerRayBlock : public IRayBlock
		{
			Type type() const override;
			void write(std::ofstream stream) override;
			size_t byteSize() override;

			std::vector<std::vector<Segment>> segments;
			std::vector<Vec3> intersection_normal;
			std::vector<double> intensity;
		};

		struct OuterRayBlock : public IRayBlock
		{
			Type type() const override;
			void write(std::ofstream stream) override;
			size_t byteSize() override;
			std::vector<Segment> segments;
			std::vector<Vec3> intersection_normal;
			std::vector<double> intensity;
		};

		class RayTracingStepRayData
		{
		public:
			void write(std::ofstream stream);
		private:
			std::vector<IRayBlock> blocks;
			std::vector<size_t> block_num_ray;
		};

	}
}

#endif