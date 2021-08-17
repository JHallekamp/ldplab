#ifndef LDPLAB_DEBUG
#define LDPLAB_DEBUG

#include <iomanip>
#include <fstream>
#include <functional>
#include <sstream>
#include <vector>

#include <LDPLAB/UID.hpp>

class DebugContext
{
public:
	DebugContext(
		const std::string& file, 
		std::function<std::vector<std::string>()> ctx_swithc_method);
	void write(std::function<std::vector<std::string>()> get_str);
private:
	static void setContext(DebugContext& ctx);
	void write(const std::string& str, bool indent);
private:
	ldplab::UID<DebugContext> m_uid;
	std::function<std::vector<std::string>()> m_ctx_switch_method;
	std::ofstream m_file;
};

#endif