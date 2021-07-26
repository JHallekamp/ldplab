#ifndef WWU_LDPLAB_TOOLS_RAY_PATH_COLLECTOR_HPP
#define WWU_LDPLAB_TOOLS_RAY_PATH_COLLECOTR_HPP

namespace ldplab
{
	namespace tools
	{
		class RayPathCollector
		{
		public:
			static RayPathCollector& instance();
			void init();
			void Add();
			void nextRay();
		private:
			RayPathCollector();
		};
	}
}

#endif
