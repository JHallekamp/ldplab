#include "Debug.hpp"

#include <limits>

DebugContext::DebugContext(
	const std::string& file,
	std::function<std::vector<std::string>()> ctx_swithc_method)
	:
	m_file{ file, std::ios::app },
	m_ctx_switch_method{ ctx_swithc_method }
{ }

void DebugContext::write(std::function<std::vector<std::string>()> get_str)
{
	setContext(*this);
	std::vector<std::string> lines = get_str();
	for (size_t i = 0; i < lines.size(); ++i)
		write(lines[i], true);
}

void DebugContext::setContext(DebugContext& ctx)
{
	static size_t cur_ctx_uid = std::numeric_limits<size_t>::max();
	if (cur_ctx_uid == ctx.m_uid)
		return;
	// Context switch
	cur_ctx_uid = ctx.m_uid;
	std::stringstream ss;
	ss << "Debug context switch to " << cur_ctx_uid;
	ctx.write(ss.str(), false);
	std::vector<std::string> lines = ctx.m_ctx_switch_method();
	for (size_t i = 0; i < lines.size(); ++i)
		ctx.write(lines[i], false);
}

void DebugContext::write(const std::string& str, bool indent)
{
	if (indent)
		m_file << "    ";
	m_file << str << std::endl;
}
