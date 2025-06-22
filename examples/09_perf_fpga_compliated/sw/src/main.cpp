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

#define DEF_N_REPS 64

// #define DETAIL_REPORT

// Registers, corresponding to registers defined the vFPGA
enum class BenchmarkRegisters: uint32_t {
    RESET_REG        = 0,     // Reset register
    N_REPS_REG       = 1,     // Total number of transfers to perform
    DONE_REG         = 2,     // Number of transfers done
    REQ_CTRL_REG     = 3,     // Control register for a single request
    REQ_N_BEATS_REG  = 4,     // Number of beats for a single request
    REQ_LEN_A_REG    = 5,     // Length A of the request
    REQ_LEN_B_REG    = 6,     // Length B of the request
    REQ_VADDR_A_REG  = 7,     // Virtual address A of the request
    REQ_VADDR_B_REG  = 8,     // Virtual address B of the request
    REQ_PID_REG      = 9,     // Process ID of the request
    TIMER_REG        = 10,    // Timer register to measure the time of the operation
};

// 01 written to REQ_CTRL_REG starts a read operation and 10 written to CTRL registers starts a write
enum class BenchmarkOperation: uint8_t {
    START_RD = 0x1,
    START_WR = 0x2
};

void print_regs(std::unique_ptr<coyote::cThread<std::any>> &coyote_thread, char *header) {
    std::cout << "-----" << header << "-----" << std::endl;
    std::cout << "RESET_REG:       " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::RESET_REG)) << std::endl;
    std::cout << "N_REPS_REG:      " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::N_REPS_REG)) << std::endl;
    std::cout << "DONE_REG:        " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::DONE_REG)) << std::endl;
    std::cout << "REQ_CTRL_REG:    " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_CTRL_REG)) << std::endl;
    std::cout << "REQ_N_BEATS_REG: " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_N_BEATS_REG)) << std::endl;
    std::cout << "REQ_LEN_A_REG:   " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_LEN_A_REG)) << std::endl;
    std::cout << "REQ_LEN_B_REG:   " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_LEN_B_REG)) << std::endl;
    std::cout << "REQ_VADDR_A_REG: " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_VADDR_A_REG)) << std::endl;
    std::cout << "REQ_VADDR_B_REG: " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_VADDR_B_REG)) << std::endl;
    std::cout << "REQ_PID_REG:     " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_PID_REG)) << std::endl;
    std::cout << "TIMER_REG:       " << coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::TIMER_REG)) << std::endl;
    std::cout << std::endl;
}

double run_bench(
    std::unique_ptr<coyote::cThread<std::any>> &coyote_thread, 
    unsigned int size_a, int* mem_a[4],
    unsigned int size_b, int* mem_b[4],
    unsigned int n_reps, unsigned int n_runs, BenchmarkOperation oper, bool mapped
) {
    // printf("Running benchmark with %s operation, %u runs, %u repetitions, size A: %u, size B: %u, mapped: %d\n", 
    //        (oper == BenchmarkOperation::START_RD ? "READ" : "WRITE"), n_runs, n_reps, size_a, size_b, mapped);

    // Randomly initialise the data
    for (int i = 0; i < size_a / sizeof(int); i++) {
        for (int j = 0; j < 4; ++j) {
            mem_a[j][i] = i + j;
        }
    }
    for (int i = 0; i < size_b / sizeof(int); i++) {
        for (int j = 0; j < 4; ++j) {
            mem_b[j][i] = i + j;
        }
    }

    // Single iteration of transfers reads or writes
    auto benchmark_run = [&]() {
        // print_regs(coyote_thread, "Before RESET");

        // Reset the benchmark
        coyote_thread->setCSR(1, static_cast<uint32_t>(BenchmarkRegisters::RESET_REG));
        while (coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::RESET_REG)) != 0) {}

        // print_regs(coyote_thread, "After RESET");

        // Set the number of transfers to perform
        coyote_thread->setCSR(n_reps, static_cast<uint32_t>(BenchmarkRegisters::N_REPS_REG));

        // print_regs(coyote_thread, "After N_REPS_REG");

        // Calculate the number of beats required per operation
        uint64_t n_beats = ((size_a + 64 - 1) / 64) + ((size_b + 64 - 1) / 64);

        for (int i = 0; i < n_reps; ++i) {
            // Set n_beats
            coyote_thread->setCSR(n_beats, static_cast<uint32_t>(BenchmarkRegisters::REQ_N_BEATS_REG));

            // Set request length and addr
            coyote_thread->setCSR(size_a, static_cast<uint32_t>(BenchmarkRegisters::REQ_LEN_A_REG));
            coyote_thread->setCSR(size_b, static_cast<uint32_t>(BenchmarkRegisters::REQ_LEN_B_REG));
            coyote_thread->setCSR((uint64_t) mem_a[i % 4], static_cast<uint32_t>(BenchmarkRegisters::REQ_VADDR_A_REG));
            coyote_thread->setCSR((uint64_t) mem_b[i % 4], static_cast<uint32_t>(BenchmarkRegisters::REQ_VADDR_B_REG));

            // Set the process ID
            coyote_thread->setCSR(coyote_thread->getCtid(), static_cast<uint32_t>(BenchmarkRegisters::REQ_PID_REG));

            // print_regs(coyote_thread, "Before REQ_CTRL_REG");

            // Set the operation type and start
            coyote_thread->setCSR(static_cast<uint32_t>(oper), static_cast<uint32_t>(BenchmarkRegisters::REQ_CTRL_REG));
            while (coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::REQ_CTRL_REG)) != 0) {}

            // print_regs(coyote_thread, "After REQ_CTRL_REG");
        }
        
        // Wait until done register is asserted high
        while (coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::DONE_REG)) < n_reps) {}

        // print_regs(coyote_thread, "After DONE_REG");

        // Read from time register and convert to ns
        return (double) coyote_thread->getCSR(static_cast<uint32_t>(BenchmarkRegisters::TIMER_REG)) * (double) CLOCK_PERIOD_NS;
    };

    // Run benchmark
    double avg_time = 0.0;
    for(int k = 0; k < n_runs; k++) {
        // Clear TLB
        for (int i = 0; i < size_a; i += 0x8000) {
            for (int j = 0; j < 4; ++j) {
                coyote_thread->userUnmap((uint8_t *)mem_a[j] + i);
            }
        }
        for (int i = 0; i < size_b; i += 0x8000) {
            for (int j = 0; j < 4; ++j) {
                coyote_thread->userUnmap((uint8_t *)mem_b[j] + i);
            }
        }
        // Pre-fault the memory, if required
        if (mapped) {
            for (int j = 0; j < 4; ++j) {
                // printf("Pre-faulting memory A[%d]:%p and B[%d]:%p\n", j, mem_a[j], j, mem_b[j]);
                coyote_thread->userMapComplex(mem_a[j], size_a, true, 1);
                coyote_thread->userMapComplex(mem_b[j], size_b, true, 0);
            }
        }

        // printf("Pre-process finished, starting RUN No.%d\n", k);
        // while (getchar() != '\n');

        // Run the benchmark
        double tmp_time = benchmark_run();
#ifdef DETAIL_REPORT
        std::cout << tmp_time << ", ";
#endif
        avg_time += tmp_time;

        // printf("Benchmark finished\n");
        // while (getchar() != '\n');
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
    unsigned int n_runs;
    unsigned int a_size, b_size;
    bool hugepages;
    bool mapped;
    unsigned int n_reps;

    boost::program_options::options_description runtime_options("Coyote Perf FPGA Options");
    runtime_options.add_options()
        ("operation,o", boost::program_options::value<bool>(&operation)->default_value(false), "Benchmark operation: READ(0) or WRITE(1)")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(100), "Number of times to repeat the test")
        ("a_size,a", boost::program_options::value<unsigned int>(&a_size)->default_value(4 * 1024), "Memory size of A")
        ("b_size,b", boost::program_options::value<unsigned int>(&b_size)->default_value(4 * 1024 * 1024), "Memory size of B")
        ("hugepages,h", boost::program_options::value<bool>(&hugepages)->default_value(false), "Use hugepages")
        ("mapped,m", boost::program_options::value<bool>(&mapped)->default_value(true), "Use mapped memory")
        ("n_reps,n", boost::program_options::value<unsigned int>(&n_reps)->default_value(DEF_N_REPS), "Number of throughput repetitions");
    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);
    BenchmarkOperation oper = operation ? BenchmarkOperation::START_WR : BenchmarkOperation::START_RD;

    PR_HEADER("CLI PARAMETERS:");
    std::cout << "Benchmark operation: " << (operation ? "WRITE" : "READ") << std::endl;
    std::cout << "Number of test runs: " << n_runs << std::endl;
    std::cout << "Memory size of A: " << a_size << std::endl;
    std::cout << "Memory size of B: " << b_size << std::endl;
    std::cout << "Enable hugepages: " << hugepages << std::endl;
    std::cout << "Enable mapped pages: " << mapped << std::endl;
    std::cout << "Number of transfer repetitions: " << n_reps << std::endl;
    std::cout << std::endl;

    // Create Coyote thread and allocate source & destination memory
    std::unique_ptr<coyote::cThread<std::any>> coyote_thread(
        new coyote::cThread<std::any>(DEFAULT_VFPGA_ID, getpid(), 0)
    );
    
    // We need 4 stream for this benchmark
    coyote_thread->setNStrm(4);
    if (hugepages) {
        coyote_thread->setDTlbPgsize(21);
    } else {
        coyote_thread->setDTlbPgsize(12);
    }

    int* mem_a[4];
    int* mem_b[4];
    if (hugepages) {
        for (int i = 0; i < 4; ++i) {
            mem_a[i] = (int *) mmap(NULL, a_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
            mem_b[i] = (int *) mmap(NULL, b_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
            if (!mem_a[i] || !mem_b[i]) { throw std::runtime_error("Could not allocate memory; exiting..."); }
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            mem_a[i] = (int *) aligned_alloc(coyote::pageSize, a_size); 
            mem_b[i] = (int *) aligned_alloc(coyote::pageSize, b_size);
            if (!mem_a[i] || !mem_b[i]) { throw std::runtime_error("Could not allocate memory; exiting..."); }
        }
    }

    // Benchmark sweep
    PR_HEADER("PERF FPGA");

    // Run throughput test
    if (n_reps != 0) {
        double throughput_time = run_bench(coyote_thread, a_size, mem_a, b_size, mem_b, n_reps, n_runs, oper, mapped);
        double throughput_a = ((double) n_reps * (double) a_size) / (1024.0 * 1024.0 * throughput_time * 1e-9);
        double throughput_b = ((double) n_reps * (double) b_size) / (1024.0 * 1024.0 * throughput_time * 1e-9);
        std::cout << "Size A: " << std::setw(8) << a_size << " Bytes; ";
        std::cout << "Average throughput A: " << std::setw(8) << throughput_a << " MB/s; ";
        std::cout << "Size B: " << std::setw(8) << b_size << " Bytes; ";
        std::cout << "Average throughput B: " << std::setw(8) << throughput_b << " MB/s" << std::endl;
    }


    if (hugepages) {
        for (int i = 0; i < 4; ++i) {
            munmap(mem_a[i], a_size);
            munmap(mem_b[i], b_size);
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            free(mem_a[i]);
            free(mem_b[i]);
        }
    }
    for (int i = 0; i < a_size; i += 0x8000) {
        for (int j = 0; j < 4; ++j) {
            coyote_thread->userUnmap((uint8_t *)mem_a[j] + i);
        }
    }
    for (int i = 0; i < b_size; i += 0x8000) {
        for (int j = 0; j < 4; ++j) {
            coyote_thread->userUnmap((uint8_t *)mem_b[j] + i);
        }
    }


    return EXIT_SUCCESS;
}

