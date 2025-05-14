// Includes
#include <any>
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/program_options.hpp>

#include "cThread.hpp"

// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0

#define DEF_N_LATENCY_REPS 1
#define DEF_N_THROUGHPUT_REPS 64

// #define DETAIL_REPORT

// Registers, corresponding to registers defined the vFPGA
enum class BenchmarkRegisters: uint32_t {
    CTRL_REG = 0,           // AP start, read or write
    DONE_REG = 1,           // Number of completed requests
    TIMER_REG = 2,          // Timer register
    VADDR_REG = 3,          // Buffer virtual address
    LEN_REG = 4,            // Buffer length (size in bytes)
    PID_REG = 5,            // Coyote thread ID
    N_REPS_REG = 6,         // Number of read/write repetitions
    N_BEATS_REG = 7         // Number of expected AXI beats
};

// 01 written to CTRL_REG starts a read operation and 10 written to CTRL registers starts a write
enum class BenchmarkOperation: uint8_t {
    START_RD = 0x1,
    START_WR = 0x2
};

double run_bench(
    std::unique_ptr<coyote::cThread<std::any>> &coyote_thread, unsigned int size, 
    int *mem, unsigned int transfers, unsigned int n_runs, BenchmarkOperation oper,
    bool mapped, int tlb_type
) {
    // Randomly initialise the data
    for (int i = 0; i < size / sizeof(int); i++) {
        mem[i] = rand() % 1024 - 512;     
    }

    // Single iteration of transfers reads or writes
    auto benchmark_run = [&]() {
        // Set the required registers from SW
        uint64_t n_beats = transfers * ((size + 64 - 1) / 64);
        coyote_thread->setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(BenchmarkRegisters::VADDR_REG));
        coyote_thread->setCSR(size, static_cast<uint32_t>(BenchmarkRegisters::LEN_REG));
        coyote_thread->setCSR(coyote_thread->getCtid(), static_cast<uint32_t>(BenchmarkRegisters::PID_REG));
        coyote_thread->setCSR(transfers, static_cast<uint32_t>(BenchmarkRegisters::N_REPS_REG));
        coyote_thread->setCSR(n_beats, static_cast<uint32_t>(BenchmarkRegisters::N_BEATS_REG));

        // Start the operation by writing to the control register
        coyote_thread->setCSR(static_cast<uint64_t>(oper), static_cast<uint32_t>(BenchmarkRegisters::CTRL_REG));
        
        // Wait until done register is asserted high
        while (coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::DONE_REG)) < transfers) {}

        // Read from time register and convert to ns
        return (double) coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::TIMER_REG)) * (double) CLOCK_PERIOD_NS;
    };

    // Run benchmark
    double avg_time = 0.0;
    for(int j = 0; j < n_runs; j++) {
        // Clear TLB
        for (int i = 0; i < size; i += 0x8000) {
            coyote_thread->userUnmap((uint8_t *)mem + i);
        }
        // Pre-fault the memory, if required
        if (mapped) {
            coyote_thread->userMapComplex(mem, size, 1, tlb_type);
        }
        // Run the benchmark
        double tmp_time = benchmark_run();
#ifdef DETAIL_REPORT
        std::cout << tmp_time << ", ";
#endif
        avg_time += tmp_time;
    }
    
#ifdef DETAIL_REPORT
    std::cout << std::endl;
#endif

    avg_time = avg_time / (double) n_runs;

    return avg_time;
}

int main(int argc, char *argv[]) {
    // CLI arguments
    bool operation;
    unsigned int n_runs, min_size, max_size;
    bool hugepages;
    bool mapped;
    unsigned int n_throughput_reps, n_latency_reps;
    int tlb_type;

    boost::program_options::options_description runtime_options("Coyote Perf FPGA Options");
    runtime_options.add_options()
        ("operation,o", boost::program_options::value<bool>(&operation)->default_value(false), "Benchmark operation: READ(0) or WRITE(1)")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(100), "Number of times to repeat the test")
        ("min_size,x", boost::program_options::value<unsigned int>(&min_size)->default_value(64), "Starting (minimum) transfer size")
        ("max_size,X", boost::program_options::value<unsigned int>(&max_size)->default_value(16 * 1024 * 1024), "Ending (maximum) transfer size")
        ("hugepages,h", boost::program_options::value<bool>(&hugepages)->default_value(true), "Use hugepages")
        ("mapped,m", boost::program_options::value<bool>(&mapped)->default_value(true), "Use mapped memory")
        ("n_throughput_reps,nt", boost::program_options::value<unsigned int>(&n_throughput_reps)->default_value(DEF_N_THROUGHPUT_REPS), "Number of throughput repetitions")
        ("n_latency_reps,nl", boost::program_options::value<unsigned int>(&n_latency_reps)->default_value(DEF_N_LATENCY_REPS), "Number of latency repetitions")
        ("tlb,t", boost::program_options::value<int>(&tlb_type)->default_value(-1), "TLB Type (-1: auto, 0: stream, 1: discrete)");
    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);
    BenchmarkOperation oper = operation ? BenchmarkOperation::START_WR : BenchmarkOperation::START_RD;

    PR_HEADER("CLI PARAMETERS:");
    std::cout << "Benchmark operation: " << (operation ? "WRITE" : "READ") << std::endl;
    std::cout << "Number of test runs: " << n_runs << std::endl;
    std::cout << "Starting transfer size: " << min_size << std::endl;
    std::cout << "Ending transfer size: " << max_size << std::endl;
    std::cout << "Enable hugepages: " << hugepages << std::endl;
    std::cout << "Enable mapped pages: " << mapped << std::endl;
    std::cout << "Number of throughput repetitions: " << n_throughput_reps << std::endl;
    std::cout << "Number of latency repetitions: " << n_latency_reps << std::endl;
    std::cout << "TLB Type: " << (tlb_type == -1 ? "AUTO" : (tlb_type == 0 ? "STREAM" : "DISCRETE")) << std::endl;
    std::cout << std::endl;

    // Create Coyote thread and allocate source & destination memory
    std::unique_ptr<coyote::cThread<std::any>> coyote_thread(
        new coyote::cThread<std::any>(DEFAULT_VFPGA_ID, getpid(), 0)
    );
    
    // We need use one stream for this benchmark
    coyote_thread->setNStrm(1);

    int* mem;
    if (hugepages) {
        mem = (int *) mmap(NULL, max_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
    } else {
        mem = (int *) aligned_alloc(coyote::pageSize, max_size);
    }
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Benchmark sweep
    PR_HEADER("PERF FPGA");
    unsigned int curr_size = min_size;
    while(curr_size <= max_size) {
        std::cout << "Size: " << std::setw(8) << curr_size << "; ";
        
        // Run throughput test
        if (n_throughput_reps != 0) {
            double throughput_time = run_bench(coyote_thread, curr_size, mem, n_throughput_reps, n_runs, oper, mapped, tlb_type);
            double throughput = ((double) n_throughput_reps * (double) curr_size) / (1024.0 * 1024.0 * throughput_time * 1e-9);
            std::cout << "Average throughput: " << std::setw(8) << throughput << " MB/s; ";
        }
        
        // Run latency test
        if (n_latency_reps != 0) {
            double latency_time = run_bench(coyote_thread, curr_size, mem, n_latency_reps, n_runs, oper, mapped, tlb_type);
            std::cout << "Average latency: " << std::setw(8) << latency_time / 1e3 << " us" << std::endl;
        } else {
            std::cout << std::endl;
        }

        // Update size and proceed to next iteration
        curr_size *= 2;
    }

    if (hugepages) {
        munmap(mem, max_size);
    } else {
        free(mem);
    }
    for (int i = 0; i < max_size; i += 0x8000) {
        coyote_thread->userUnmap((uint8_t *)mem + i);
    }
    

    return EXIT_SUCCESS;
}

