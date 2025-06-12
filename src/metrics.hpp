#pragma once

#include <chrono>
#include <mutex>
#include <vector>
#include <atomic>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <fstream>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>

class Metrics {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;

    Metrics(bool enableLogging = true, bool enableMetrics = true)
        : enableLogging(enableLogging), enableMetrics(enableMetrics),
          totalSuccess(0), totalFail(0) {}

    TimePoint start() const {
        if (!enableMetrics) return TimePoint();
        return Clock::now();
    }

    void end(TimePoint startTime, bool success) {
        if (!enableMetrics) return;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            Clock::now() - startTime
        ).count();

        std::lock_guard<std::mutex> lock(dataMutex);
        latencies.push_back(duration);
        results.push_back(success);
        if (success)
            ++totalSuccess;
        else
            ++totalFail;

        if (enableLogging) {
            std::cout << "[Benchmark] Request took " << duration << "us ("
                      << (success ? "success" : "fail") << ")\n";
        }
    }

    void reset() {
        std::lock_guard<std::mutex> lock(dataMutex);
        latencies.clear();
        results.clear();
        totalSuccess = 0;
        totalFail = 0;
    }

    struct Stats {
        double minLatency = 0;
        double maxLatency = 0;
        double averageLatency = 0;
        double p95Latency = 0;
        double p99Latency = 0;
        size_t successCount = 0;
        size_t failCount = 0;
        double successRate = 0;
    };

    Stats getStats() {
        std::lock_guard<std::mutex> lock(dataMutex);
        Stats stats{};
        size_t count = latencies.size();
        if (count == 0) return stats;

        std::vector<int64_t> sortedLatencies = latencies;
        std::sort(sortedLatencies.begin(), sortedLatencies.end());

        stats.minLatency = sortedLatencies.front();
        stats.maxLatency = sortedLatencies.back();
        stats.averageLatency = std::accumulate(
            sortedLatencies.begin(), sortedLatencies.end(), 0.0
        ) / count;

        stats.p95Latency = percentile(sortedLatencies, 0.95);
        stats.p99Latency = percentile(sortedLatencies, 0.99);
        stats.successCount = totalSuccess;
        stats.failCount = totalFail;
        stats.successRate = (totalSuccess + totalFail) > 0
            ? static_cast<double>(totalSuccess) / (totalSuccess + totalFail)
            : 0.0;

        return stats;
    }

    void printStats() {
        if (!enableMetrics) {
            std::cout << "[Benchmark] Metrics collection is disabled.\n";
            return;
        }

        Stats stats = getStats();
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "\n--- Benchmark Results ---\n";
        std::cout << "Min latency   : " << stats.minLatency/1000 << " ms\n";
        std::cout << "Max latency   : " << stats.maxLatency/1000 << " ms\n";
        std::cout << "Avg latency   : " << stats.averageLatency/1000 << " ms\n";
        std::cout << "P95 latency   : " << stats.p95Latency/1000 << " ms\n";
        std::cout << "P99 latency   : " << stats.p99Latency/1000 << " ms\n";
        std::cout << "Success count : " << stats.successCount << "\n";
        std::cout << "Fail count    : " << stats.failCount << "\n";
        std::cout << "Success rate  : " << stats.successRate * 100 << " %\n";
    }

    void exportCSV(const std::string& filename) {
        if (!enableMetrics) {
            std::cerr << "[Benchmark] Metrics disabled, skipping CSV export.\n";
            return;
        }

        std::lock_guard<std::mutex> lock(dataMutex);
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "[Benchmark] Failed to open CSV file: " << filename << "\n";
            return;
        }

        out << "latency_ms,success\n";
        for (size_t i = 0; i < latencies.size(); ++i) {
            out << latencies[i]/1000 << "," << (results[i] ? "1" : "0") << "\n";
        }

        std::cout << "[Benchmark] Exported " << latencies.size()
                  << " entries to: " << filename << "\n";
    }

    void setLogging(bool enabled) { enableLogging = enabled; }
    void setMetricsEnabled(bool enabled) { enableMetrics = enabled; }

private:
    mutable std::mutex dataMutex;
    std::vector<int64_t> latencies;
    std::vector<bool> results;
    std::atomic<size_t> totalSuccess;
    std::atomic<size_t> totalFail;
    bool enableLogging;
    bool enableMetrics;

    static double percentile(const std::vector<int64_t>& sorted, double p) {
        if (sorted.empty()) return 0;
        size_t index = static_cast<size_t>(p * sorted.size());
        index = std::min(index, sorted.size() - 1);
        return static_cast<double>(sorted[index]);
    }
};
