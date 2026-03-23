#include <chrono>
#include <iostream>

#include "hw_cli/command_parser.hpp"
#include "hw_cli/hw_agent.hpp"
#include "hw_cli/json_builder.hpp"

using namespace hw_cli;

int main(int argc, char** argv) {
    CommandLineArgs args;
    std::string err;
    if (!ParseArgs(argc, argv, args, err)) {
        std::cerr << err << std::endl;
        return 2;
    }

    const auto t0 = std::chrono::steady_clock::now();
    ResultPayload out;
    out.mode = args.verb;
    HwAgent agent;

    // 默认最大反馈年龄 100ms
    const int max_age_ms = 100;

    if (args.verb == "ping") {
        out.ok = true;
    } else if (args.verb == "enable") {
        out.ok = agent.EnsureEnabled(args.low_gain, args.auto_report, err);
    } else if (args.verb == "goto_offset") {
        out.ok = agent.MoveToOffsets(args.seconds, args.low_gain, args.auto_report, err);
        if (out.ok) {
            // 移动完成后采样一次状态
            std::string sample_err;
            agent.GetState(args.sample_delay_ms, max_age_ms, out, sample_err);
        }
    } else if (args.verb == "disable") {
        out.ok = agent.DisableAll(err);
    } else if (args.verb == "hold") {
        out.ok = agent.HoldOffsets(args.low_gain, args.auto_report, args.sample_delay_ms, max_age_ms, err);
        // HoldOffsets 内部已经采样，不需要再调用 GetState
    } else if (args.verb == "get_state") {
        // 纯查询，不改变电机状态
        out.ok = agent.GetState(args.sample_delay_ms, max_age_ms, out, err);
    } else if (args.verb == "enable_auto_report") {
        out.ok = agent.EnableAutoReport(err);
    } else if (args.verb == "disable_auto_report") {
        out.ok = agent.DisableAutoReport(err);
    } else if (args.verb == "set") {
        JointArray raw{};
        if (!ParseActionCsv(args.action_csv, raw, err)) {
            out.ok = false;
        } else {
            out.ok = agent.SendRawAction(raw,
                                         args.ensure_enabled,
                                         args.low_gain,
                                         args.auto_report,
                                         args.sample_delay_ms,
                                         max_age_ms,
                                         out,
                                         err);
        }
    } else {
        err = "unknown verb: " + args.verb;
        out.ok = false;
    }

    out.error = err;
    out.rtt_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count());

    std::cout << ToJson(out) << std::endl;
    return out.ok ? 0 : 1;
}